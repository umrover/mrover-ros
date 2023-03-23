#pragma once

#include <sl/Camera.hpp>

#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

class ZedNode {
private:
    ros::NodeHandle mNh, mPnh;
    sl::Camera mZed;
    tf2_ros::Buffer mTfBuffer;
    tf2_ros::TransformListener mTfListener;
    tf2_ros::TransformBroadcaster mTfBroadcaster;
    image_transport::ImageTransport mIt;
    ros::Publisher mPcPub;
    image_transport::Publisher mLeftImgPub;

    size_t mUpdateTick = 0;
    sensor_msgs::Image mLeftImgMsg;
    sensor_msgs::PointCloud2 mPointCloudMsg;

    sl::Resolution mImageResolution{1280, 720};
    sl::Mat mImageMat;
    sl::Mat mPointCloudXYZMat;
    sl::Mat mPointCloudNormalMat;

    std::thread mUpdateThread;

public:
    ZedNode(ros::NodeHandle const& nh = {}, ros::NodeHandle const& pnh = {"~"});

    ~ZedNode();

    void update();
};