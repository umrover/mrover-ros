#include "invariant_ekf.hpp"
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

class InvariantEKFNode {
private:
    ros::NodeHandle mNh, mPnh;
    ros::Subscriber mImuSub, mGpsSub;
    ros::Publisher mOdometryPub;
    InvariantEKF mEKF;

    void imu_callback(const sensor_msgs::Imu& msg);

    void gps_callback(const geometry_msgs::Pose& msg);

public:
    InvariantEKFNode();

    InvariantEKFNode(const InvariantEKFNode&) = delete;
    InvariantEKFNode& operator=(const InvariantEKFNode&) = delete;
};