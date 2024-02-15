#include "simulator.hpp"

namespace mrover {

    auto SimulatorNodelet::cameraUpdate(Camera& camera, wgpu::CommandEncoder& encoder, wgpu::RenderPassDescriptor const& passDescriptor) -> void {
        {
            wgpu::RenderPassEncoder colorPass = encoder.beginRenderPass(passDescriptor);
            colorPass.setPipeline(mPbrPipeline);

            if (!camera.sceneUniforms.buffer) camera.sceneUniforms.init(mDevice);
            camera.sceneUniforms.value.lightColor = {1, 1, 1, 1};
            camera.sceneUniforms.value.lightInWorld = {0, 0, 5, 1};
            float aspect = static_cast<float>(camera.resolution.x()) / static_cast<float>(camera.resolution.y());
            camera.sceneUniforms.value.cameraToClip = computeCameraToClip(mFovDegrees * DEG_TO_RAD, aspect, NEAR, FAR).cast<float>();
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
        {
            wgpu::ComputePassEncoder computePass = encoder.beginComputePass();
            computePass.setPipeline(mPointCloudPipeline);

            if (!camera.computeUniforms.buffer) camera.computeUniforms.init(mDevice);
            camera.computeUniforms.value.resolution = camera.resolution;
            camera.computeUniforms.value.clipToCamera = camera.sceneUniforms.value.cameraToClip.inverse();
            camera.computeUniforms.enqueueWrite();

            std::size_t pointCloudBufferSize = camera.resolution.x() * camera.resolution.y() * sizeof(Point);
            if (!camera.pointCloudBuffer) {
                {
                    wgpu::BufferDescriptor descriptor;
                    descriptor.usage = wgpu::BufferUsage::Storage | wgpu::BufferUsage::CopySrc;
                    descriptor.size = pointCloudBufferSize;
                    camera.pointCloudBuffer = mDevice.createBuffer(descriptor);
                }
                {
                    wgpu::BufferDescriptor descriptor;
                    descriptor.usage = wgpu::BufferUsage::CopyDst | wgpu::BufferUsage::MapRead;
                    descriptor.size = pointCloudBufferSize;
                    camera.pointCloudBufferStaging = mDevice.createBuffer(descriptor);
                }
            }

            std::array<wgpu::BindGroupEntry, 5> entries;
            entries[0].binding = 0;
            entries[0].buffer = camera.computeUniforms.buffer;
            entries[0].size = sizeof(ComputeUniforms);
            entries[1].binding = 1;
            entries[1].textureView = camera.colorTextureView;
            entries[2].binding = 2;
            entries[2].textureView = camera.normalTextureView;
            entries[3].binding = 3;
            entries[3].textureView = camera.depthTextureView;
            entries[4].binding = 4;
            entries[4].buffer = camera.pointCloudBuffer;
            entries[4].size = camera.resolution.x() * camera.resolution.y() * sizeof(Point);
            wgpu::BindGroupDescriptor descriptor;
            descriptor.layout = mPointCloudPipeline.getBindGroupLayout(0);
            descriptor.entryCount = entries.size();
            descriptor.entries = entries.data();
            wgpu::BindGroup bindGroup = mDevice.createBindGroup(descriptor);
            computePass.setBindGroup(0, bindGroup, 0, nullptr);

            computePass.dispatchWorkgroups(camera.resolution.x(), camera.resolution.y(), 1);

            computePass.end();

            encoder.copyBufferToBuffer(camera.pointCloudBuffer, 0, camera.pointCloudBufferStaging, 0, camera.pointCloudBuffer.getSize());

            bindGroup.release();
            computePass.release();
        }
    }

    auto cartesianToGeodetic(R3 const& cartesian, Eigen::Vector3d const& referenceGeodetic, double referenceHeadingDegrees) -> Eigen::Vector3d {
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

    auto computeNavSatFix(SE3d const& gpsInMap, Eigen::Vector3d const& referenceGeodetic, double referenceHeadingDegrees) -> sensor_msgs::NavSatFix {
        sensor_msgs::NavSatFix gpsMessage;
        gpsMessage.header.stamp = ros::Time::now();
        gpsMessage.header.frame_id = "map";
        auto geodetic = cartesianToGeodetic(gpsInMap.translation(), referenceGeodetic, referenceHeadingDegrees);
        gpsMessage.latitude = geodetic(0);
        gpsMessage.longitude = geodetic(1);
        gpsMessage.altitude = geodetic(2);
        return gpsMessage;
    }

    auto computeImu(SE3d const& imuInMap, R3 const& imuAngularVelocity, R3 const& linearAcceleration, R3 const& magneticField) -> ImuAndMag {
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
        if (auto lookup = getUrdf("rover"); lookup) {
            URDF const& rover = *lookup;

            if (mLinearizedPosePub) {
                SE3d baseLinkInMap = rover.linkInWorld("base_link");
                geometry_msgs::PoseWithCovarianceStamped pose;
                pose.header.stamp = ros::Time::now();
                pose.header.frame_id = "map";
                R3 p = baseLinkInMap.translation();
                pose.pose.pose.position.x = p.x();
                pose.pose.pose.position.y = p.y();
                pose.pose.pose.position.z = p.z();
                S3 q = baseLinkInMap.quat();
                pose.pose.pose.orientation.w = q.w();
                pose.pose.pose.orientation.x = q.x();
                pose.pose.pose.orientation.y = q.y();
                pose.pose.pose.orientation.z = q.z();
                // TODO(quintin, riley): fill in covariance
                mLinearizedPosePub.publish(pose);
            }

            if (mGpsTask.shouldUpdate()) {
                SE3d leftGpsInMap = rover.linkInWorld("left_gps");
                mLeftGpsPub.publish(computeNavSatFix(leftGpsInMap, mGpsLinerizationReferencePoint, mGpsLinerizationReferenceHeading));

                SE3d rightGpsInMap = rover.linkInWorld("right_gps");
                mRightGpsPub.publish(computeNavSatFix(rightGpsInMap, mGpsLinerizationReferencePoint, mGpsLinerizationReferenceHeading));
            }
            if (mImuTask.shouldUpdate()) {
                R3 imuAngularVelocity = btVector3ToR3(rover.physics->getBaseOmega());
                R3 roverLinearVelocity = btVector3ToR3(rover.physics->getBaseVel());
                R3 imuLinearAcceleration = (roverLinearVelocity - mRoverLinearVelocity) / std::chrono::duration_cast<std::chrono::duration<float>>(dt).count();
                mRoverLinearVelocity = roverLinearVelocity;
                SE3d imuInMap = rover.linkInWorld("imu");
                mImuPub.publish(computeImu(imuInMap, imuAngularVelocity, imuLinearAcceleration, imuInMap.rotation().matrix().transpose().col(1)));
            }
        }
    }

} // namespace mrover
