#include <ros/init.h>
#include <ros/ros.h>
#include <terrain_particle_filter.hpp>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
#include <iostream>

class FilterNode {
private:
    ros::NodeHandle mNh;
    ros::Publisher mPosePub;
    ros::Subscriber mGroundTruthSub, mCmdVelSub;
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
        std::cout << "Predicting" << std::endl;
        Eigen::Vector3d velCmd(msg.linear.x, msg.linear.y, msg.angular.z);
        auto now = std::chrono::system_clock::now();
        double dt = std::chrono::duration<double>(now - mLastTime).count();
        mFilter.predict(velCmd, dt);
    }

    void publish_pose() {
        if (!mInitialized) return;
        std::cout << "Publishing pose" << std::endl;
        auto pose = mFilter.get_pose_estimate();
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.pose.position.x = pose.x();
        msg.pose.position.y = pose.y();

        tf2::Quaternion q;
        q.setRPY(0, 0, pose.angle());
        msg.pose.orientation = tf2::toMsg(q);
        mPosePub.publish(msg);
    }

public:
    FilterNode() : mFilter("terrain.pcd", 0.1, 0.1, 0.1, 0.1) {
        std::cout << "FilterNode constructor" << std::endl;
        mNumParticles = 1;
        mPosePub = mNh.advertise<geometry_msgs::PoseStamped>("pf_pose", 1);
        ros::Subscriber mGroundTruthSub = mNh.subscribe("ground_truth", 1, &FilterNode::ground_truth_callback, this);
        ros::Subscriber mVelCmdSub = mNh.subscribe("cmd_vel", 1, &FilterNode::cmd_vel_callback, this);
        // ros::Subscriber mVelCmdSub = mNh.subscribe("cmd_vel", 1, &[](const geometry_msgs::Twist& msg) {
        //     std::cout << "cmd_vel callback" << std::endl;
        // });
        mLastTime = std::chrono::system_clock::now();
    }
    
    void spin() {
        ros::Rate rate(10);
        while (ros::ok()) {
            std::cout << "Spinning" << std::endl;
            ros::spinOnce();
            publish_pose();
            rate.sleep();
        }
    }
};