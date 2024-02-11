#include "invariant_ekf.hpp"
#include <chrono>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <boost_cpp23_workaround.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>



using TimePoint = std::chrono::time_point<std::chrono::system_clock>;
using Duration = std::chrono::duration<double>;

class InvariantEKFNode {
private:
    ros::NodeHandle mNh, mPnh;
    ros::Subscriber mImuSub, mGpsSub, mMagSub;
    ros::Publisher mOdometryPub;
    tf2_ros::TransformBroadcaster TfBroadcaster;
    tf2_ros::Buffer TfBuffer;
    tf2_ros::TransformListener TfListener{TfBuffer};
    InvariantEKF mEKF;
    TimePoint mLastImuTime, mLastGpsTime, mLastMagTime;

    InvariantEKF init_EKF();

    void imu_callback(const sensor_msgs::Imu& msg);

    void gps_callback(const geometry_msgs::Pose& msg);

    void mag_callback(const sensor_msgs::MagneticField& msg);

    void publish_odometry();

    void publish_tf();

public:
    int main();

    InvariantEKFNode();

    InvariantEKFNode(const InvariantEKFNode&) = delete;
    InvariantEKFNode& operator=(const InvariantEKFNode&) = delete;

    void run();
};