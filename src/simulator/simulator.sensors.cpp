#include "simulator.hpp"

namespace mrover {

    auto SimulatorNodelet::cameraUpdate(Camera& camera, wgpu::RenderPassEncoder& pass) -> void {
        pass.setPipeline(mPbrPipeline);

        SE3 cameraInWorld = btTransformToSe3(camera.link->m_cachedWorldTransform);

        if (!camera.sceneUniforms.buffer) {
            camera.sceneUniforms.init(mDevice);
            camera.sceneUniforms.value.lightColor = {1, 1, 1, 1};
            camera.sceneUniforms.value.lightInWorld = {0, 0, 5, 1};
        }

        float aspect = static_cast<float>(camera.resolution.x()) / static_cast<float>(camera.resolution.y());
        camera.sceneUniforms.value.cameraToClip = computeCameraToClip(mFovDegrees * DEG_TO_RAD, aspect, NEAR, FAR).cast<float>();
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
        pass.setBindGroup(1, bindGroup, 0, nullptr);

        renderModels(pass);

        // // TODO(quintin): Change this
        // SE3 cameraInWorld = btTransformToSe3(camera.link->m_cachedWorldTransform);
        // mPbrProgram.uniform("cameraInWorld", cameraInWorld.position().cast<float>());
        // mPbrProgram.uniform("worldToCamera", ROS_TO_GL * cameraInWorld.matrix().inverse().cast<float>());

        // glBindFramebuffer(GL_FRAMEBUFFER, camera.framebufferHandle);
        // glViewport(0, 0, camera.resolution.width, camera.resolution.height);
        // glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        // renderModels(true);

        // if (mPointCloudProgram.handle == GL_INVALID_HANDLE) return;
        // if (camera.pcPub.getNumSubscribers() == 0) return;
        // if (!camera.updateTask.shouldUpdate()) return;

        // auto pointCloud = boost::make_shared<sensor_msgs::PointCloud2>();
        // pointCloud->is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
        // pointCloud->is_dense = true;
        // pointCloud->width = camera.resolution.width;
        // pointCloud->height = camera.resolution.height;
        // pointCloud->header.stamp = ros::Time::now();
        // pointCloud->header.frame_id = "zed2i_left_camera_frame";
        // fillPointCloudMessageHeader(pointCloud);

        // glUseProgram(mPointCloudProgram.handle);

        // Eigen::Matrix4f clipToCamera = ROS_TO_GL.inverse() * cameraToClip.inverse();
        // mPointCloudProgram.uniform("clipToCamera", clipToCamera);
        // mPointCloudProgram.uniform("resolution", Eigen::Vector2i{camera.resolution.width, camera.resolution.height});

        // glActiveTexture(GL_TEXTURE0);
        // glBindTexture(GL_TEXTURE_2D, camera.colorTextureHandle);

        // glActiveTexture(GL_TEXTURE1);
        // glBindTexture(GL_TEXTURE_2D, camera.depthTextureHandle);

        // glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, camera.pointCloudArrayHandle);

        // glDispatchCompute(camera.resolution.width, camera.resolution.height, 1);
        // glMemoryBarrier(GL_ALL_BARRIER_BITS);
        // glBindBuffer(GL_SHADER_STORAGE_BUFFER, camera.pointCloudArrayHandle);
        // GLvoid* data = glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_READ_ONLY);
        // assert(data);
        // std::memcpy(pointCloud->data.data(), data, pointCloud->data.size());
        // glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);

        // camera.pcPub.publish(pointCloud);
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
