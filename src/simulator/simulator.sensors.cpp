#include "simulator.hpp"

namespace mrover {

    auto SimulatorNodelet::camerasUpdate(Camera& camera) -> void {
        if (camera.pcPub.getNumSubscribers() == 0) return;

        if (Clock::now() - camera.lastUpdate < std::chrono::duration_cast<Clock::duration>(std::chrono::duration<float>{1.0f / camera.rate})) return;

        camera.lastUpdate = Clock::now();

        glBindTexture(GL_TEXTURE_2D, camera.colorTextureHandle);
        glGetTexImage(GL_TEXTURE_2D, 0, GL_BGR, GL_UNSIGNED_BYTE, camera.colorImage.data);
        flip(camera.colorImage, camera.colorImage, 1);

        glBindTexture(GL_TEXTURE_2D, camera.depthTextureHandle);
        glGetTexImage(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, GL_FLOAT, camera.depthImage.data);
        flip(camera.depthImage, camera.depthImage, 1);

        mPointCloud->is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
        mPointCloud->is_dense = true;
        mPointCloud->width = camera.resolution.width;
        mPointCloud->height = camera.resolution.height;
        mPointCloud->header.stamp = ros::Time::now();
        mPointCloud->header.frame_id = "zed2i_left_camera_frame";
        fillPointCloudMessageHeader(mPointCloud);
        assert(mPointCloud->data.size() == camera.colorImage.total() * sizeof(Point));

        float f1 = std::tan(mFov * DEG2RAD / 2.0f) * 2.0f;
        float f2 = f1 * static_cast<float>(camera.resolution.width) / static_cast<float>(camera.resolution.height);

        auto* points = reinterpret_cast<Point*>(mPointCloud->data.data());
        camera.colorImage.forEach<cv::Vec3b>([&](cv::Vec3b& pixel, int const* position) -> void {
            float depth = 2 * camera.depthImage.at<float>(position) - 1;

            int ri = position[0], ci = position[1];

            auto& [x, y, z, b, g, r, a, normal_x, normal_y, normal_z, curvature] = points[ri * mPointCloud->width + ci];
            b = pixel[0];
            g = pixel[1];
            r = pixel[2];
            x = -2 * NEAR * FAR / (depth * (FAR - NEAR) - (FAR + NEAR));
            y = (static_cast<float>(ci) / static_cast<float>(camera.resolution.width) - 0.5f) * f1;
            z = (static_cast<float>(ri) / static_cast<float>(camera.resolution.height) - 0.5f) * f2;
            normal_x = std::numeric_limits<float>::quiet_NaN();
            normal_y = std::numeric_limits<float>::quiet_NaN();
            normal_z = std::numeric_limits<float>::quiet_NaN();
            curvature = std::numeric_limits<float>::quiet_NaN();
        });
        camera.pcPub.publish(mPointCloud);
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
