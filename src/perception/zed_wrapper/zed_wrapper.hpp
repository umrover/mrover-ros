#pragma once

#include <condition_variable>

#include <sl/Camera.hpp>

#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <tag_detector.hpp>

class ZedNode {
private:
    ros::NodeHandle mNh, mPnh;

    tf2_ros::Buffer mTfBuffer;
    tf2_ros::TransformListener mTfListener;
    tf2_ros::TransformBroadcaster mTfBroadcaster;
    image_transport::ImageTransport mIt;
    ros::Publisher mPcPub;
    image_transport::Publisher mLeftImgPub;

    sensor_msgs::Image mLeftImgMsg;
    sensor_msgs::PointCloud2Ptr mGrabPointCloud;
    sensor_msgs::PointCloud2Ptr mTagPointCloud;

    int mResolution{};
    int mGrabTargetFps{};
    int mImageWidth{};
    int mImageHeight{};

    sl::Camera mZed;
    sl::Mat mLeftImageMat;
    sl::Mat mPointCloudXYZMat;
    sl::Mat mPointCloudNormalMat;

    std::thread mTagThread;
    std::thread mGrabThread;
    bool mIsGrabDone = false;
    std::condition_variable mGrabDone;
    std::mutex mSwapPcMutex;

    std::unique_ptr<TagDetectorNode> mTagDetectorNode;

    size_t mUpdateTick = 0;

public:
    ZedNode(ros::NodeHandle const& nh = {}, ros::NodeHandle const& pnh = {"~"});

    ~ZedNode();

    void grabUpdate();

    void tagUpdate();
};