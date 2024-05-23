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

    auto cartesianToGeodetic(R3d const& cartesian, R3d const& referenceGeodetic, double referenceHeadingDegrees) -> R3d {
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

    auto computeNavSatFix(R3d const& gpsInMap, R3d const& referenceGeodetic, double referenceHeadingDegrees) -> sensor_msgs::NavSatFix {
        sensor_msgs::NavSatFix gpsMessage;
        gpsMessage.header.stamp = ros::Time::now();
        gpsMessage.header.frame_id = "map";
        auto geodetic = cartesianToGeodetic(gpsInMap, referenceGeodetic, referenceHeadingDegrees);
        gpsMessage.latitude = geodetic(0);
        gpsMessage.longitude = geodetic(1);
        gpsMessage.altitude = geodetic(2);
        return gpsMessage;
    }

    auto computeImu(SO3d const& imuInMap, R3d const& imuAngularVelocity, R3d const& linearAcceleration) -> sensor_msgs::Imu {
        sensor_msgs::Imu imuMessage;
        imuMessage.header.stamp = ros::Time::now();
        imuMessage.header.frame_id = "base_link";
        S3d q = imuInMap.quat();
        imuMessage.orientation.w = q.w();
        imuMessage.orientation.x = q.x();
        imuMessage.orientation.y = q.y();
        imuMessage.orientation.z = q.z();
        imuMessage.angular_velocity.x = imuAngularVelocity.x();
        imuMessage.angular_velocity.y = imuAngularVelocity.y();
        imuMessage.angular_velocity.z = imuAngularVelocity.z();
        imuMessage.linear_acceleration.x = linearAcceleration.x();
        imuMessage.linear_acceleration.y = linearAcceleration.y();
        imuMessage.linear_acceleration.z = linearAcceleration.z();
        return imuMessage;
    }

    auto btVector3ToR3(btVector3 const& v) -> R3d {
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
                R3d p = baseLinkInMap.translation();
                odometry.pose.pose.position.x = p.x();
                odometry.pose.pose.position.y = p.y();
                odometry.pose.pose.position.z = p.z();
                S3d q = baseLinkInMap.quat();
                odometry.pose.pose.orientation.w = q.w();
                odometry.pose.pose.orientation.x = q.x();
                odometry.pose.pose.orientation.y = q.y();
                odometry.pose.pose.orientation.z = q.z();
                R3d v = btVector3ToR3(rover.physics->getBaseVel());
                odometry.twist.twist.linear.x = v.x();
                odometry.twist.twist.linear.y = v.y();
                odometry.twist.twist.linear.z = v.z();
                R3d w = btVector3ToR3(rover.physics->getBaseOmega());
                odometry.twist.twist.angular.x = w.x();
                odometry.twist.twist.angular.y = w.y();
                odometry.twist.twist.angular.z = w.z();
                mGroundTruthPub.publish(odometry);
            }
            for (auto& [link, updateTask, pub]: mGps) {
                if (!updateTask.shouldUpdate()) continue;

                R3d gpsInMap = btTransformToSe3(link->m_cachedWorldTransform).translation();
                R3d gpsNoise{mGPSDist(mRNG), mGPSDist(mRNG), mGPSDist(mRNG)};
                gpsInMap += gpsNoise;
                pub.publish(computeNavSatFix(gpsInMap, mGpsLinearizationReferencePoint, mGpsLinerizationReferenceHeading));
            }
            for (Imu imu: mImus) {
                if (!imu.updateTask.shouldUpdate()) continue;

                auto dtS = std::chrono::duration_cast<std::chrono::duration<double>>(dt).count();
                R3d roverAngularVelocity = btVector3ToR3(rover.physics->getBaseOmega());
                R3d roverLinearVelocity = btVector3ToR3(rover.physics->getBaseVel());
                R3d roverLinearAcceleration = (roverLinearVelocity - mRoverLinearVelocity) / dtS;
                mRoverLinearVelocity = roverLinearVelocity;
                SO3d imuInMap = btTransformToSe3(imu.link->m_cachedWorldTransform).asSO3();
                R3d roverMagVector = imuInMap.inverse().rotation().col(1);

                R3d
                        accelNoise{mAccelDist(mRNG), mAccelDist(mRNG), mAccelDist(mRNG)},
                        gyroNoise{mGyroDist(mRNG), mGyroDist(mRNG), mGyroDist(mRNG)},
                        magNoise{mMagDist(mRNG), mMagDist(mRNG), mMagDist(mRNG)};
                roverLinearAcceleration += accelNoise;
                roverAngularVelocity += gyroNoise;
                roverMagVector += magNoise;

                imu.pub.publish(computeImu(imuInMap, roverAngularVelocity, roverLinearAcceleration));

                constexpr double SEC_TO_MIN = 1.0 / 60.0;
                mOrientationDrift += mOrientationDriftRate * SEC_TO_MIN * dtS;
                SO3d::Tangent orientationNoise;
                orientationNoise << mRollDist(mRNG), mPitchDist(mRNG), mYawDist(mRNG);
                imuInMap += orientationNoise + mOrientationDrift;

                imu.uncalibPub.publish(computeImu(imuInMap, roverAngularVelocity, roverLinearAcceleration));

                CalibrationStatus calibrationStatus;
                calibrationStatus.header.stamp = ros::Time::now();
                calibrationStatus.header.frame_id = "base_link";
                calibrationStatus.magnetometer_calibration = 3;
                imu.calibStatusPub.publish(calibrationStatus);

                sensor_msgs::MagneticField field;
                field.header.stamp = ros::Time::now();
                field.header.frame_id = "base_link";
                field.magnetic_field.x = roverMagVector.x();
                field.magnetic_field.y = roverMagVector.y();
                field.magnetic_field.z = roverMagVector.z();
                imu.magPub.publish(field);
            }
        }
    }

    auto SimulatorNodelet::motorStatusUpdate() -> void {
        if (auto lookup = getUrdf("rover"); lookup) {
            URDF const& rover = *lookup;

            for (MotorGroup& motorGroup: mMotorGroups) {
                ControllerState controllerState;

                sensor_msgs::JointState jointState;
                jointState.header.stamp = ros::Time::now();

                for (std::string const& msgName: motorGroup.names) {
                    std::string urdfName = msgToUrdf.forward(msgName).value();

                    controllerState.name.push_back(msgName);
                    controllerState.state.emplace_back("Armed");
                    controllerState.error.emplace_back("None");
                    std::uint8_t limitSwitches = 0b000;
                    if (auto limits = rover.model.getLink(urdfName)->parent_joint->limits) {
                        double jointPosition = rover.physics->getJointPos(rover.linkNameToMeta.at(urdfName).index);
                        constexpr double OFFSET = 0.05;
                        if (jointPosition < limits->lower + OFFSET) limitSwitches |= 0b001;
                        if (jointPosition > limits->upper - OFFSET) limitSwitches |= 0b010;
                    }
                    controllerState.limit_hit.push_back(limitSwitches);

                    jointState.name.push_back(msgName);
                    jointState.position.push_back(rover.physics->getJointPos(rover.linkNameToMeta.at(urdfName).index));
                    jointState.velocity.push_back(rover.physics->getJointVel(rover.linkNameToMeta.at(urdfName).index));
                    jointState.effort.push_back(rover.physics->getJointTorque(rover.linkNameToMeta.at(urdfName).index));
                }

                motorGroup.controllerStatePub.publish(controllerState);
                motorGroup.jointStatePub.publish(jointState);
            }
        }
    }

} // namespace mrover