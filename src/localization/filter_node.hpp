#include <ros/init.h>
#include <ros/ros.h>
#include <terrain_particle_filter.hpp>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
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
    ros::Publisher mPosePub, mTerrainPub, mNeighborhoodPub, mNormalPub, mNormalsPub, mParticlesPub;
    ros::Subscriber mGroundTruthSub, mVelCmdSub, mImuSub;
    TerrainParticleFilter mFilter;
    std::chrono::time_point<std::chrono::system_clock> mLastTime;
    bool mInitialized = false;
    int mNumParticles;
    std::default_random_engine mRNG;

    void ground_truth_callback(const nav_msgs::Odometry& msg) {
        auto now = std::chrono::system_clock::now();
        double dt = std::chrono::duration<double>(now - mLastTime).count();
        mLastTime = now;
        if (!mInitialized) {
            std::cout << "Initializing filter" << std::endl;
            tf2::Quaternion q;
            tf2::fromMsg(msg.pose.pose.orientation, q);
            tf2::Matrix3x3 r(q);
            double roll, pitch, yaw;
            r.getRPY(roll, pitch, yaw);
            manif::SE2d pose(msg.pose.pose.position.x, msg.pose.pose.position.y, yaw);
            mFilter.init_particles(mNumParticles);
            // mFilter.init_particles(pose, mNumParticles);
            mInitialized = true;
        }

        Eigen::Quaterniond orientation(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
        Eigen::Vector3d velocityWorld(msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z);
        Eigen::Vector3d velocityBody = orientation.inverse() * velocityWorld;
        std::normal_distribution<double> xDist(0, 0.1);
        std::normal_distribution<double> thetaDist(0, 0.1);
        Eigen::Vector3d velCmd(velocityBody.x(), velocityBody.y(), msg.twist.twist.angular.z);
        std::cout << "Vel cmd: " << velCmd.transpose() << std::endl;
        Eigen::Vector3d noise(xDist(mRNG), 0, thetaDist(mRNG));
        mFilter.predict(velCmd, dt);
    }

    void cmd_vel_callback(const geometry_msgs::Twist& msg) {
        // if (!mInitialized) return;
        // auto now = std::chrono::system_clock::now();
        // double dt = std::chrono::duration<double>(now - mLastTime).count();
        // mLastTime = now;
        // Eigen::Vector3d velCmd(msg.linear.x, msg.linear.y, msg.angular.z);
        // mFilter.predict(velCmd, dt);
    }

    void imu_callback(const sensor_msgs::Imu& msg) {
        if (!mInitialized) return;
        // std::cout << "Updating" << std::endl;
        // auto now = std::chrono::system_clock::now();
        // double dt = std::chrono::duration<double>(now - mLastTime).count();
        // mLastTime = now;
        Eigen::Quaterniond orientation(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
        Eigen::Vector3d gyro(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
        Eigen::Vector3d accel(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
        // mFilter.predict(orientation, accel, gyro, dt);
        mFilter.update(orientation);
    }

    void publish_particles() {
        if (!mInitialized) return;
        auto particles = mFilter.get_particles();
        visualization_msgs::MarkerArray msg;
        std::vector<visualization_msgs::Marker> markers;
        for (size_t i = 0; i < particles.size(); i++) {
            auto rotation = particles[i].rotation();
            Eigen::Rotation2Dd rotation2d(rotation);
            Eigen::Quaterniond q(Eigen::AngleAxisd(rotation2d.angle(), Eigen::Vector3d::UnitZ()));
            visualization_msgs::Marker marker;
            marker.header.stamp = ros::Time();
            marker.header.frame_id = "map";
            marker.ns = "particles";
            marker.id = i;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = particles[i].x();
            marker.pose.position.y = particles[i].y();
            marker.pose.position.z = 0;
            marker.pose.orientation.x = q.x();
            marker.pose.orientation.y = q.y();
            marker.pose.orientation.z = q.z();
            marker.pose.orientation.w = q.w();
            marker.scale.x = 1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.a = 0.5;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            markers.push_back(marker);
        }
        msg.markers = markers;
        mParticlesPub.publish(msg);
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

    void publish_normals() {
        auto grid = mFilter.get_terrain_grid();
        visualization_msgs::MarkerArray msg;
        std::vector<visualization_msgs::Marker> markers;
        for (size_t i = 0; i < grid.rows(); i+=10) {
            for (size_t j = 0; j < grid.cols(); j+=10) {
                Eigen::Vector2i idx = Eigen::Vector2i(i, j);
                Eigen::Vector2d position = mFilter.idx_to_position(idx);
                double height = grid(i, j);
                auto normal = mFilter.get_surface_normal(manif::SE2d(position.x(), position.y(), 0));
                if (!normal) continue;
                visualization_msgs::Marker marker;
                marker.header.stamp = ros::Time();
                marker.header.frame_id = "map";
                marker.ns = "normals";
                marker.id = i * grid.cols() + j;
                marker.type = visualization_msgs::Marker::ARROW;
                marker.action = visualization_msgs::Marker::ADD;
                marker.points.resize(2);
                marker.points[0].x = position.x();
                marker.points[0].y = position.y();
                marker.points[0].z = height;
                marker.points[1].x = position.x() + normal->x();
                marker.points[1].y = position.y() + normal->y();
                marker.points[1].z = height + normal->z();
                marker.scale.x = 0.05;
                marker.scale.y = 0.1;
                marker.scale.z = 0.5;
                marker.color.a = 1.0;
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
                markers.push_back(marker);
            }
        }
        msg.markers = markers;
        mNormalsPub.publish(msg);
    }

    // TODO: clean this up
    void publish_normal() {
        if (!mInitialized) return;
        auto pose = mFilter.get_pose_estimate();
        auto normal = mFilter.get_surface_normal(pose);
        if (!normal) return;
        
        // convert direction vector to quaternion
        Eigen::Vector3d basis2 = normal->cross(Eigen::Vector3d::UnitZ());
        Eigen::Vector3d basis3 = normal->cross(basis2);
        Eigen::Matrix3d basis = Eigen::Matrix3d::Zero();
        basis.col(0) = *normal;
        basis.col(1) = basis2;
        basis.col(2) = basis3;
        Eigen::Quaterniond q(basis);
        q.normalize();

        visualization_msgs::Marker msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "map";
        msg.ns = "normal";
        msg.id = 0;
        msg.type = visualization_msgs::Marker::ARROW;
        msg.action = visualization_msgs::Marker::ADD;
        msg.pose.position.x = pose.x();
        msg.pose.position.y = pose.y();
        msg.pose.position.z = 0;
        msg.pose.orientation.x = q.x();
        msg.pose.orientation.y = q.y();
        msg.pose.orientation.z = q.z();
        msg.pose.orientation.w = q.w();
        msg.scale.x = 1;
        msg.scale.y = 0.2;
        msg.scale.z = 0.2;
        msg.color.a = 1.0;
        msg.color.r = 1.0;
        msg.color.g = 0.0;
        msg.color.b = 0.0;
        // msg.points.resize(2);
        // msg.points[0].x = 0;
        // msg.points[0].y = 0;
        // msg.points[0].z = 0;
        // msg.points[1].x = normal.x();
        // msg.points[1].y = normal.y();
        // msg.points[1].z = 0;
        mNormalPub.publish(msg);
    }

    void publish_terrain() {
        if (!mInitialized) return;
        auto grid = mFilter.get_terrain_grid();
        PointCloud::Ptr neighborhood = mFilter.get_neighborhood();
        neighborhood->header.frame_id = "map";
        pcl_conversions::toPCL(ros::Time::now(), neighborhood->header.stamp);

        mNeighborhoodPub.publish(*neighborhood);
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
    FilterNode() : mFilter("/home/riley/catkin_ws/src/mrover/src/localization/waves.tif", 0.01, 0.03, Eigen::Vector2d(1, 1)) {
        mNumParticles = 1000;
        // Eigen::Vector2i idx(3,1);
        // Eigen::Vector2d pos = mFilter.idx_to_position(idx);
        // Eigen::Vector2i idx2 = mFilter.position_to_idx(pos);
        // std::cout << "idx: " << idx.x() << ", " << idx.y() << std::endl;
        // std::cout << "pos: " << pos.x() << ", " << pos.y() << std::endl;
        // std::cout << "idx2: " << idx2.x() << ", " << idx2.y() << std::endl;
        mPosePub = mNh.advertise<geometry_msgs::PoseStamped>("pf_pose", 1);
        mTerrainPub = mNh.advertise<PointCloud>("terrain_map", 1);
        mNeighborhoodPub = mNh.advertise<PointCloud>("neighborhood", 1);
        mNormalPub = mNh.advertise<visualization_msgs::Marker>("normal_vector", 1);
        mNormalsPub = mNh.advertise<visualization_msgs::MarkerArray>("normals", 1);
        mParticlesPub = mNh.advertise<visualization_msgs::MarkerArray>("particles", 1);
        mGroundTruthSub = mNh.subscribe("ground_truth", 1, &FilterNode::ground_truth_callback, this);
        mVelCmdSub = mNh.subscribe("cmd_vel", 1, &FilterNode::cmd_vel_callback, this);
        mImuSub = mNh.subscribe("imu/imu_only", 1, &FilterNode::imu_callback, this);
        mLastTime = std::chrono::system_clock::now();
    }
    
    void spin() {
        ros::Rate rate(10);
        while (ros::ok()) {
            // std::cout << "Spinning" << std::endl;
            ros::spinOnce();
            // publish_normals();
            publish_normal();
            publish_particles();
            publish_pose();
            publish_terrain();
            rate.sleep();
        }
    }
};