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

            std::array<wgpu::BindGroupEntry, 4> entries;
            entries[0].binding = 0;
            entries[0].buffer = camera.computeUniforms.buffer;
            entries[0].size = sizeof(ComputeUniforms);
            entries[1].binding = 1;
            entries[1].textureView = camera.colorTextureView;
            entries[2].binding = 2;
            entries[2].textureView = camera.depthTextureView;
            entries[3].binding = 3;
            entries[3].buffer = camera.pointCloudBuffer;
            entries[3].size = camera.resolution.x() * camera.resolution.y() * sizeof(Point);
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

    auto SimulatorNodelet::gpsAndImusUpdate() -> void {
        if (auto lookup = getUrdf("rover"); lookup) {
            URDF const& rover = *lookup;

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
            mPosePub.publish(pose);
        }
    }

} // namespace mrover
