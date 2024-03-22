#include "simulator.hpp"

namespace mrover {

    auto SimulatorNodelet::renderCamera(Camera& camera, wgpu::CommandEncoder& encoder, wgpu::RenderPassDescriptor const& passDescriptor) -> void {
        wgpu::RenderPassEncoder colorPass = encoder.beginRenderPass(passDescriptor);
        colorPass.setPipeline(mPbrPipeline);

        if (!camera.sceneUniforms.buffer) camera.sceneUniforms.init(mDevice);
        camera.sceneUniforms.value.lightColor = {1, 1, 1, 1};
        camera.sceneUniforms.value.lightInWorld = {0, 0, 5, 1};
        float aspect = static_cast<float>(camera.resolution.x()) / static_cast<float>(camera.resolution.y());
        camera.sceneUniforms.value.cameraToClip = computeCameraToClip(camera.fov * DEG_TO_RAD, aspect, NEAR, FAR).cast<float>();
        SE3d cameraInWorld = btTransformToSe3(camera.link->m_cachedWorldTransform);
        camera.sceneUniforms.value.worldToCamera = cameraInWorld.inverse().transform().cast<float>();
        camera.sceneUniforms.value.cameraInWorld = cameraInWorld.translation().cast<float>().homogeneous();
        camera.sceneUniforms.enqueueWrite();

        wgpu::BindGroupEntry entry;
        entry.binding = 0;
        entry.buffer = camera.sceneUniforms.buffer;
        entry.size = sizeof(SceneUniforms);
        wgpu::BindGroupDescriptor descriptor;
        descriptor.layout = mPbrPipeline.getBindGroupLayout(1);
        descriptor.entryCount = 1;
        descriptor.entries = &entry;
        wgpu::BindGroup bindGroup = mDevice.createBindGroup(descriptor);
        colorPass.setBindGroup(1, bindGroup, 0, nullptr);

        renderModels(colorPass);

        colorPass.end();

        bindGroup.release();
        colorPass.release();
    }

    auto SimulatorNodelet::computeStereoCamera(StereoCamera& stereoCamera, wgpu::CommandEncoder& encoder) -> void {
        wgpu::ComputePassEncoder computePass = encoder.beginComputePass();
        computePass.setPipeline(mPointCloudPipeline);

        if (!stereoCamera.computeUniforms.buffer) stereoCamera.computeUniforms.init(mDevice);
        stereoCamera.computeUniforms.value.resolution = stereoCamera.base.resolution;
        stereoCamera.computeUniforms.value.clipToCamera = stereoCamera.base.sceneUniforms.value.cameraToClip.inverse();
        stereoCamera.computeUniforms.enqueueWrite();

        std::size_t pointCloudBufferSize = stereoCamera.base.resolution.x() * stereoCamera.base.resolution.y() * sizeof(Point);
        if (!stereoCamera.pointCloudBuffer) {
            {
                wgpu::BufferDescriptor descriptor;
                descriptor.usage = wgpu::BufferUsage::Storage | wgpu::BufferUsage::CopySrc;
                descriptor.size = pointCloudBufferSize;
                stereoCamera.pointCloudBuffer = mDevice.createBuffer(descriptor);
            }
            {
                wgpu::BufferDescriptor descriptor;
                descriptor.usage = wgpu::BufferUsage::CopyDst | wgpu::BufferUsage::MapRead;
                descriptor.size = pointCloudBufferSize;
                stereoCamera.pointCloudStagingBuffer = mDevice.createBuffer(descriptor);
            }
        }

        std::array<wgpu::BindGroupEntry, 5> entries;
        entries[0].binding = 0;
        entries[0].buffer = stereoCamera.computeUniforms.buffer;
        entries[0].size = sizeof(ComputeUniforms);
        entries[1].binding = 1;
        entries[1].textureView = stereoCamera.base.colorTextureView;
        entries[2].binding = 2;
        entries[2].textureView = stereoCamera.base.normalTextureView;
        entries[3].binding = 3;
        entries[3].textureView = stereoCamera.base.depthTextureView;
        entries[4].binding = 4;
        entries[4].buffer = stereoCamera.pointCloudBuffer;
        entries[4].size = stereoCamera.base.resolution.x() * stereoCamera.base.resolution.y() * sizeof(Point);
        wgpu::BindGroupDescriptor descriptor;
        descriptor.layout = mPointCloudPipeline.getBindGroupLayout(0);
        descriptor.entryCount = entries.size();
        descriptor.entries = entries.data();
        wgpu::BindGroup bindGroup = mDevice.createBindGroup(descriptor);
        computePass.setBindGroup(0, bindGroup, 0, nullptr);

        computePass.dispatchWorkgroups(stereoCamera.base.resolution.x(), stereoCamera.base.resolution.y(), 1);

        computePass.end();

        encoder.copyBufferToBuffer(stereoCamera.pointCloudBuffer, 0, stereoCamera.pointCloudStagingBuffer, 0, stereoCamera.pointCloudBuffer.getSize());

        bindGroup.release();
        computePass.release();
    }

    auto cartesianToGeodetic(R3 const& cartesian, R3 const& referenceGeodetic, double referenceHeadingDegrees) -> R3 {
        constexpr double equatorialRadius = 6378137.0;
        constexpr double flattening = 1.0 / 298.257223563;
        constexpr double eccentricity2 = 2 * flattening - flattening * flattening;
        using std::sin, std::cos, std::pow, std::sqrt;

        double lat0 = referenceGeodetic(0) * DEG_TO_RAD;
        double lon0 = referenceGeodetic(1) * DEG_TO_RAD;
        double h0 = referenceGeodetic(2);
        double temp = 1.0 / (1.0 - eccentricity2 * sin(lat0) * sin(lat0));
        double primeVerticalRadius = equatorialRadius * sqrt(temp);
        double radiusNorth = primeVerticalRadius * (1 - eccentricity2) * temp;
        double radiusEast = primeVerticalRadius * cos(lat0);

        double referenceHeadingRadians = referenceHeadingDegrees * DEG_TO_RAD;
        double lat = (lat0 + (cos(referenceHeadingRadians) * cartesian.x() + sin(referenceHeadingRadians) * cartesian.y()) / radiusNorth) / DEG_TO_RAD;
        double lon = (lon0 - (-sin(referenceHeadingRadians) * cartesian.x() + cos(referenceHeadingRadians) * cartesian.y()) / radiusEast) / DEG_TO_RAD;
        double alt = h0 + cartesian.z();
        return {lat, lon, alt};
    }

    auto computeNavSatFix(R3 const& gpsInMap, R3 const& referenceGeodetic, double referenceHeadingDegrees) -> sensor_msgs::NavSatFix {
        sensor_msgs::NavSatFix gpsMessage;
        gpsMessage.header.stamp = ros::Time::now();
        gpsMessage.header.frame_id = "map";
        auto geodetic = cartesianToGeodetic(gpsInMap, referenceGeodetic, referenceHeadingDegrees);
        gpsMessage.latitude = geodetic(0);
        gpsMessage.longitude = geodetic(1);
        gpsMessage.altitude = geodetic(2);
        return gpsMessage;
    }

    auto computeImu(SO3d const& imuInMap, R3 const& imuAngularVelocity, R3 const& linearAcceleration, R3 const& magneticField) -> ImuAndMag {
        ImuAndMag imuMessage;
        imuMessage.header.stamp = ros::Time::now();
        imuMessage.header.frame_id = "map";
        S3 q = imuInMap.quat();
        imuMessage.imu.orientation.w = q.w();
        imuMessage.imu.orientation.x = q.x();
        imuMessage.imu.orientation.y = q.y();
        imuMessage.imu.orientation.z = q.z();
        imuMessage.imu.angular_velocity.x = imuAngularVelocity.x();
        imuMessage.imu.angular_velocity.y = imuAngularVelocity.y();
        imuMessage.imu.angular_velocity.z = imuAngularVelocity.z();
        imuMessage.imu.linear_acceleration.x = linearAcceleration.x();
        imuMessage.imu.linear_acceleration.y = linearAcceleration.y();
        imuMessage.imu.linear_acceleration.z = linearAcceleration.z();
        imuMessage.mag.magnetic_field.x = magneticField.x();
        imuMessage.mag.magnetic_field.y = magneticField.y();
        imuMessage.mag.magnetic_field.z = magneticField.z();
        return imuMessage;
    }

    auto btVector3ToR3(btVector3 const& v) -> R3 {
        return {v.x(), v.y(), v.z()};
    }

    auto SimulatorNodelet::gpsAndImusUpdate(Clock::duration dt) -> void {
        if (auto lookup = getUrdf("rover")) {
            URDF const& rover = *lookup;

            {
                SE3d baseLinkInMap = rover.linkInWorld("base_link");
                nav_msgs::Odometry odometry;
                odometry.header.stamp = ros::Time::now();
                odometry.header.frame_id = "map";
                R3 p = baseLinkInMap.translation();
                odometry.pose.pose.position.x = p.x();
                odometry.pose.pose.position.y = p.y();
                odometry.pose.pose.position.z = p.z();
                S3 q = baseLinkInMap.quat();
                odometry.pose.pose.orientation.w = q.w();
                odometry.pose.pose.orientation.x = q.x();
                odometry.pose.pose.orientation.y = q.y();
                odometry.pose.pose.orientation.z = q.z();
                R3 v = btVector3ToR3(rover.physics->getBaseVel());
                odometry.twist.twist.linear.x = v.x();
                odometry.twist.twist.linear.y = v.y();
                odometry.twist.twist.linear.z = v.z();
                R3 w = btVector3ToR3(rover.physics->getBaseOmega());
                odometry.twist.twist.angular.x = w.x();
                odometry.twist.twist.angular.y = w.y();
                odometry.twist.twist.angular.z = w.z();
                mGroundTruthPub.publish(odometry);
            }
            if (mGpsTask.shouldUpdate()) {
                R3 leftGpsInMap = rover.linkInWorld("left_gps").translation();
                R3 rightGpsInMap = rover.linkInWorld("right_gps").translation();
                R3 leftGpsNoise{mGPSDist(mRNG), mGPSDist(mRNG), mGPSDist(mRNG)},
                        rightGpsNoise{mGPSDist(mRNG), mGPSDist(mRNG), mGPSDist(mRNG)};
                leftGpsInMap += leftGpsNoise;
                rightGpsInMap += rightGpsNoise; 

                RTKNavSatFix leftGpsMsg;
                RTKNavSatFix rightGpsMsg;

                leftGpsMsg.coord = computeNavSatFix(leftGpsInMap, mGpsLinearizationReferencePoint, mGpsLinerizationReferenceHeading);
                rightGpsMsg.coord = computeNavSatFix(rightGpsInMap, mGpsLinearizationReferencePoint, mGpsLinerizationReferenceHeading);

                if (mRTkFixFreq(mRNG) == 1) {
                    leftGpsMsg.fix_type = RTKNavSatFix::RTK_FIX;
                    rightGpsMsg.fix_type = RTKNavSatFix::RTK_FIX;
                }
                else if (mRTkFixFreq(mRNG) == 2 || mRTkFixFreq(mRNG) == 3) {
                    leftGpsMsg.fix_type = RTKNavSatFix::FLOATING_FIX;
                    rightGpsMsg.fix_type = RTKNavSatFix::FLOATING_FIX;
                }
                else {
                    leftGpsMsg.fix_type = RTKNavSatFix::NO_FIX;
                    rightGpsMsg.fix_type = RTKNavSatFix::NO_FIX;
                }

                leftGpsMsg.header = leftGpsMsg.coord.header;
                rightGpsMsg.header = leftGpsMsg.coord.header;

                mLeftGpsPub.publish(leftGpsMsg);
                mRightGpsPub.publish(rightGpsMsg);
            }
            if (mImuTask.shouldUpdate()) {
                auto dt_s = std::chrono::duration_cast<std::chrono::duration<double>>(dt).count();
                R3 roverAngularVelocity = btVector3ToR3(rover.physics->getBaseOmega());
                R3 roverLinearVelocity = btVector3ToR3(rover.physics->getBaseVel());
                R3 roverLinearAcceleration = (roverLinearVelocity - mRoverLinearVelocity) / dt_s;
                mRoverLinearVelocity = roverLinearVelocity;
                SO3d imuInMap = rover.linkInWorld("imu").asSO3();
                R3 roverMagVector = imuInMap.inverse().rotation().col(1);

                R3 accelNoise{mAccelDist(mRNG), mAccelDist(mRNG), mAccelDist(mRNG)},
                        gyroNoise{mGyroDist(mRNG), mGyroDist(mRNG), mGyroDist(mRNG)},
                        magNoise{mMagDist(mRNG), mMagDist(mRNG), mMagDist(mRNG)};
                roverLinearAcceleration += accelNoise;
                roverAngularVelocity += gyroNoise;
                roverMagVector += magNoise;

                constexpr double SEC_TO_MIN = 1.0 / 60.0;
                mOrientationDrift += mOrientationDriftRate * SEC_TO_MIN * dt_s;
                SO3d::Tangent orientationNoise;
                orientationNoise << mRollDist(mRNG), mPitchDist(mRNG), mYawDist(mRNG);
                imuInMap += orientationNoise + mOrientationDrift;

                mImuPub.publish(computeImu(imuInMap, roverAngularVelocity, roverLinearAcceleration, roverMagVector));
            }
        }
    }

    auto SimulatorNodelet::motorStatusUpdate() -> void {
        if (auto lookup = getUrdf("rover"); lookup) {
            URDF const& rover = *lookup;

            MotorsStatus status;
            status.joint_states.header.stamp = ros::Time::now();
            status.joint_states.header.frame_id = "map";
            ControllerState driveControllerState;
            for (auto& position: {"front", "center", "back"}) {
                for (auto& side: {"left", "right"}) {
                    std::string linkName = std::format("{}_{}_wheel_link", position, side);
                    int index = rover.linkNameToMeta.at(linkName).index;
                    double pos = rover.physics->getJointPos(index);
                    double vel = rover.physics->getJointVel(index);
                    double torque = rover.physics->getJointTorque(index);

                    status.name.push_back(linkName);
                    status.joint_states.name.push_back(linkName);
                    status.joint_states.position.push_back(pos);
                    status.joint_states.velocity.push_back(vel);
                    status.joint_states.effort.push_back(torque);

                    status.moteus_states.name.push_back(linkName);
                    status.moteus_states.state.emplace_back("Armed");
                    status.moteus_states.error.emplace_back("None");

                    driveControllerState.name.push_back(linkName);
                    driveControllerState.state.emplace_back("Armed");
                    driveControllerState.error.emplace_back("None");
                    driveControllerState.limit_hit.push_back(0b000);
                }
            }
            mMotorStatusPub.publish(status);
            mDriveControllerStatePub.publish(driveControllerState);

            ControllerState armControllerState;
            for (auto& linkName: {"arm_a_link", "arm_b_link", "arm_c_link", "arm_d_link", "arm_e_link"}) {
                armControllerState.name.emplace_back(armMsgToUrdf.backward(linkName).value());
                armControllerState.state.emplace_back("Armed");
                armControllerState.error.emplace_back("None");

                std::uint8_t limitSwitches = 0b000;
                if (auto limits = rover.model.getLink(linkName)->parent_joint->limits) {
                    double joinPosition = rover.physics->getJointPos(rover.linkNameToMeta.at(linkName).index);
                    constexpr double OFFSET = 0.05;
                    if (joinPosition < limits->lower + OFFSET) limitSwitches |= 0b001;
                    if (joinPosition > limits->upper - OFFSET) limitSwitches |= 0b010;
                }
                armControllerState.limit_hit.push_back(limitSwitches);
            }
            mArmControllerStatePub.publish(armControllerState);
        }
    }

} // namespace mrover
