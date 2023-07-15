#include <ros/init.h>
#include <ros/ros.h>
#include <terrain_particle_filter.hpp>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <chrono>
#include <iostream>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

class FilterNode {
private:
    ros::NodeHandle mNh;
    ros::Publisher mPosePub, mTerrainPub;
    ros::Subscriber mGroundTruthSub, mVelCmdSub;
    TerrainParticleFilter mFilter;
    std::chrono::time_point<std::chrono::system_clock> mLastTime;
    bool mInitialized = false;
    int mNumParticles;

    void ground_truth_callback(const nav_msgs::Odometry& msg) {
        if (!mInitialized) {
            std::cout << "Initializing filter" << std::endl;
            tf2::Quaternion q;
            tf2::fromMsg(msg.pose.pose.orientation, q);
            tf2::Matrix3x3 r(q);
            double roll, pitch, yaw;
            r.getRPY(roll, pitch, yaw);
            manif::SE2d pose(msg.pose.pose.position.x, msg.pose.pose.position.y, yaw);
            mFilter.init_particles(pose, mNumParticles);
            mInitialized = true;
        }
    }

    void cmd_vel_callback(const geometry_msgs::Twist& msg) {
        if (!mInitialized) return;
        // std::cout << "Predicting" << std::endl;
        Eigen::Vector3d velCmd(msg.linear.x, msg.linear.y, msg.angular.z);
        auto now = std::chrono::system_clock::now();
        double dt = std::chrono::duration<double>(now - mLastTime).count();
        mFilter.predict(velCmd, dt);
    }

    void publish_pose() {
        if (!mInitialized) return;
        auto pose = mFilter.get_pose_estimate();
        // std::cout << "Publishing pose: " << pose.x() << ", " << pose.y() << ", " << pose.angle() << std::endl;
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "map";
        msg.pose.position.x = pose.x();
        msg.pose.position.y = pose.y();

        tf2::Quaternion q;
        q.setRPY(0, 0, pose.angle());
        msg.pose.orientation = tf2::toMsg(q);
        mPosePub.publish(msg);
    }

    void publish_terrain() {
        if (!mInitialized) return;
        auto grid = mFilter.get_terrain_grid();

        // TODO: is pointer necessary? emplace_back?
        PointCloud::Ptr pclCloud(new PointCloud);
        for (size_t i = 0; i < grid.rows(); i++) {
            for (size_t j = 0; j < grid.cols(); j++) {
                pcl::PointXYZ point;
                Eigen::Vector2d pos = mFilter.idx_to_position(Eigen::Vector2i(i, j));
                point.x = pos.x();
                point.y = pos.y();
                point.z = grid(i, j);
                pclCloud->points.push_back(point);
            }
        }
        pclCloud->header.frame_id = "map";
        pclCloud->width = pclCloud->points.size();
        pclCloud->height = 1;
        pcl_conversions::toPCL(ros::Time::now(), pclCloud->header.stamp);

        mTerrainPub.publish(*pclCloud);
    }

public:
    FilterNode() : mFilter("/home/riley/catkin_ws/src/mrover/src/localization/terrain.tif", 0, 0, Eigen::Vector2d(10, 10)) {
        // std::cout << "FilterNode constructor" << std::endl;
        mNumParticles = 1;
        Eigen::Vector2i idx(3,1);
        Eigen::Vector2d pos = mFilter.idx_to_position(idx);
        Eigen::Vector2i idx2 = mFilter.position_to_idx(pos);
        std::cout << "idx: " << idx.x() << ", " << idx.y() << std::endl;
        std::cout << "pos: " << pos.x() << ", " << pos.y() << std::endl;
        std::cout << "idx2: " << idx2.x() << ", " << idx2.y() << std::endl;
        mPosePub = mNh.advertise<geometry_msgs::PoseStamped>("pf_pose", 1);
        mTerrainPub = mNh.advertise<PointCloud>("terrain_map", 1);
        mGroundTruthSub = mNh.subscribe("ground_truth", 1, &FilterNode::ground_truth_callback, this);
        mVelCmdSub = mNh.subscribe("cmd_vel", 1, &FilterNode::cmd_vel_callback, this);
        // ros::Subscriber mVelCmdSub = mNh.subscribe("cmd_vel", 1, &[](const geometry_msgs::Twist& msg) {
        //     std::cout << "cmd_vel callback" << std::endl;
        // });
        mLastTime = std::chrono::system_clock::now();
    }
    
    void spin() {
        ros::Rate rate(10);
        while (ros::ok()) {
            // std::cout << "Spinning" << std::endl;
            ros::spinOnce();
            publish_pose();
            publish_terrain();
            rate.sleep();
        }
    }
};