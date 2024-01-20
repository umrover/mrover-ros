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
            SE3 cameraInWorld = btTransformToSe3(camera.link->m_cachedWorldTransform);
            camera.sceneUniforms.value.worldToCamera = cameraInWorld.matrix().inverse().cast<float>();
            camera.sceneUniforms.value.cameraInWorld = cameraInWorld.position().cast<float>().homogeneous();
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

    Eigen::Vector3d cartesian_to_geodetic(R3 const& cartesian, Eigen::Vector3d const& ref_geodetic, double ref_heading) {
        constexpr double equatorial_radius = 6378137.0;
        constexpr double flattening = 1.0 / 298.257223563;
        constexpr double eccentricity2 = 2 * flattening - flattening * flattening;
        using std::sin, std::cos, std::pow, std::sqrt, std::numbers::pi;

        auto lat0 = ref_geodetic(0);
        auto lon0 = ref_geodetic(1);
        auto h0 = ref_geodetic(2);
        double temp = 1.0 / (1.0 - eccentricity2 * sin(lat0 * pi / 180.0) * sin(lat0 * pi / 180.0));
        double prime_vertical_radius = equatorial_radius * sqrt(temp);
        double radius_north = prime_vertical_radius * (1 - eccentricity2) * temp;
        double radius_east = prime_vertical_radius * cos(lat0 * pi / 180.0);

        double lat = lat0 + (cos(ref_heading) * cartesian.x() + sin(ref_heading) * cartesian.y()) / radius_north * 180.0 / pi;
        double lon = lon0 - (-sin(ref_heading) * cartesian.x() + cos(ref_heading) * cartesian.y()) / radius_east * 180.0 / pi;
        double alt = h0 + cartesian.z();
        return {lat, lon, alt};
    }

    auto computeNavSatFix(SE3 const& gpuInMap, Eigen::Vector3d const& ref_geodetic, double ref_heading) -> sensor_msgs::NavSatFix {
        sensor_msgs::NavSatFix gps_msg;
        gps_msg.header.stamp = ros::Time::now();
        gps_msg.header.frame_id = "map";
        auto geodetic = cartesian_to_geodetic(gpuInMap.position(), ref_geodetic, ref_heading);
        gps_msg.latitude = geodetic(0);
        gps_msg.longitude = geodetic(1);
        gps_msg.altitude = geodetic(2);
        return gps_msg;
    }

    auto SimulatorNodelet::gpsAndImusUpdate() -> void {
        if (auto lookup = getUrdf("rover"); lookup) {
            URDF const& rover = *lookup;

            if (mLinearizedPosePub) {
                SE3 baseLinkInMap = rover.linkInWorld("base_link");
                geometry_msgs::PoseWithCovarianceStamped pose;
                pose.header.stamp = ros::Time::now();
                pose.header.frame_id = "map";
                R3 p = baseLinkInMap.position();
                pose.pose.pose.position.x = p.x();
                pose.pose.pose.position.y = p.y();
                pose.pose.pose.position.z = p.z();
                S3 q = baseLinkInMap.rotation().quaternion();
                pose.pose.pose.orientation.w = q.w();
                pose.pose.pose.orientation.x = q.x();
                pose.pose.pose.orientation.y = q.y();
                pose.pose.pose.orientation.z = q.z();
                // TODO(quintin, riley): fill in covariance
                mLinearizedPosePub.publish(pose);
            }

            if (mLeftGpsPub) {
                SE3 leftGpsInMap = rover.linkInWorld("left_gps");
                mLeftGpsPub.publish(computeNavSatFix(leftGpsInMap));
            }
            if (mRightGpsPub) {
                SE3 rightGpsInMap = rover.linkInWorld("right_gps");
                mRightGpsPub.publish(computeNavSatFix(rightGpsInMap));
            }
        }
    }

} // namespace mrover
