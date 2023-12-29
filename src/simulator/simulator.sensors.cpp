#include "simulator.hpp"

namespace mrover {

    auto SimulatorNodelet::camerasUpdate(Camera& camera) -> void {
        glUseProgram(mPbrProgram.handle);

        float aspect = static_cast<float>(camera.resolution.width) / static_cast<float>(camera.resolution.height);
        Eigen::Matrix4f cameraToClip = perspective(mFov * DEG_TO_RAD, aspect, NEAR, FAR).cast<float>();
        mPbrProgram.uniform("cameraToClip", cameraToClip);

        SE3 cameraInWorld = btTransformToSe3(mLinkNameToRigidBody.at(camera.linkName)->getWorldTransform());
        mPbrProgram.uniform("cameraInWorld", cameraInWorld.position().cast<float>());
        mPbrProgram.uniform("worldToCamera", ROS_TO_GL * cameraInWorld.matrix().inverse().cast<float>());

        glBindFramebuffer(GL_FRAMEBUFFER, camera.framebufferHandle);
        glViewport(0, 0, camera.resolution.width, camera.resolution.height);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        renderModels(true);

        if (camera.pcPub.getNumSubscribers() == 0) return;

        if (Clock::now() - camera.lastUpdate < std::chrono::duration_cast<Clock::duration>(std::chrono::duration<float>{1.0f / camera.rate})) return;

        camera.lastUpdate = Clock::now();

        // glBindTexture(GL_TEXTURE_2D, camera.colorTextureHandle);
        // glGetTexImage(GL_TEXTURE_2D, 0, GL_BGR, GL_UNSIGNED_BYTE, camera.colorImage.data);
        // flip(camera.colorImage, camera.colorImage, 1);
        //
        // glBindTexture(GL_TEXTURE_2D, camera.depthTextureHandle);
        // glGetTexImage(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, GL_FLOAT, camera.depthImage.data);
        // flip(camera.depthImage, camera.depthImage, 1);
        //
        auto pointCloud = boost::make_shared<sensor_msgs::PointCloud2>();
        pointCloud->is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
        pointCloud->is_dense = true;
        pointCloud->width = camera.resolution.width;
        pointCloud->height = camera.resolution.height;
        pointCloud->header.stamp = ros::Time::now();
        pointCloud->header.frame_id = "zed2i_left_camera_frame";
        fillPointCloudMessageHeader(pointCloud);

        Eigen::Matrix4f clipToCamera = ROS_TO_GL.inverse() * cameraToClip.inverse();
        //
        // auto* points = reinterpret_cast<Point*>(pointCloud->data.data());
        // camera.colorImage.forEach<cv::Vec3b>([&](cv::Vec3b& pixel, int const* position) -> void {
        //     int ri = position[0], ci = position[1];
        //
        //     Eigen::Vector3f pointInClip{
        //             static_cast<float>(ci) / static_cast<float>(camera.resolution.width) * 2 - 1,
        //             static_cast<float>(ri) / static_cast<float>(camera.resolution.height) * 2 - 1,
        //             2 * camera.depthImage.at<float>(position) - 1,
        //     };
        //     Eigen::Vector3f pointInCamera = (clipToCamera * pointInClip.homogeneous()).hnormalized();
        //
        //     auto& [x, y, z, b, g, r, a, normal_x, normal_y, normal_z, curvature] = points[ri * pointCloud->width + ci];
        //     b = pixel[0];
        //     g = pixel[1];
        //     r = pixel[2];
        //     x = pointInCamera.x();
        //     y = pointInCamera.y();
        //     z = pointInCamera.z();
        //     normal_x = std::numeric_limits<float>::quiet_NaN();
        //     normal_y = std::numeric_limits<float>::quiet_NaN();
        //     normal_z = std::numeric_limits<float>::quiet_NaN();
        //     curvature = std::numeric_limits<float>::quiet_NaN();
        // });

        glUseProgram(mPointCloudProgram.handle);

        mPointCloudProgram.uniform("clipToCamera", clipToCamera);
        mPointCloudProgram.uniform("resolution", Eigen::Vector2i{camera.resolution.width, camera.resolution.height});

        glBindImageTexture(0, camera.colorTextureHandle, 0, GL_FALSE, 0, GL_READ_ONLY, GL_RGBA8);
        glBindImageTexture(1, camera.depthTextureHandle, 0, GL_FALSE, 0, GL_READ_ONLY, GL_DEPTH_COMPONENT32);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, camera.pointCloudArrayHandle);

        glDispatchCompute(camera.resolution.width, camera.resolution.height, 1);
        glMemoryBarrier(GL_ALL_BARRIER_BITS);

        glBindBuffer(GL_SHADER_STORAGE_BUFFER, camera.pointCloudArrayHandle);
        GLvoid* data = glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_READ_ONLY);
        assert(data);
        std::memcpy(pointCloud->data.data(), data, pointCloud->data.size());
        glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);

        camera.pcPub.publish(pointCloud);

        auto duration = Clock::now() - camera.lastUpdate;
        ROS_INFO_STREAM_THROTTLE(0.5, std::format("PointCloud conversion took {} ms", std::chrono::duration_cast<std::chrono::milliseconds>(duration).count()));
    }

    auto SimulatorNodelet::gpsAndImusUpdate() -> void {
        auto it = mLinkNameToRigidBody.find("rover#base_link");
        if (it == mLinkNameToRigidBody.end()) return;

        btRigidBody* baseLinkRb = it->second;
        btTransform baseLinkInMap = baseLinkRb->getWorldTransform();

        geometry_msgs::PoseWithCovarianceStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map";
        pose.pose.pose.position.x = baseLinkInMap.getOrigin().x();
        pose.pose.pose.position.y = baseLinkInMap.getOrigin().y();
        pose.pose.pose.position.z = baseLinkInMap.getOrigin().z();
        pose.pose.pose.orientation.w = baseLinkInMap.getRotation().w();
        pose.pose.pose.orientation.x = baseLinkInMap.getRotation().x();
        pose.pose.pose.orientation.y = baseLinkInMap.getRotation().y();
        pose.pose.pose.orientation.z = baseLinkInMap.getRotation().z();
        mPosePub.publish(pose);
    }

} // namespace mrover
