#include "invariant_ekf.hpp"

using TimePoint = std::chrono::time_point<std::chrono::system_clock>;
using Duration = std::chrono::duration<double>;

class InvariantEKFNode {
private:
    ros::NodeHandle mNh, mPnh;
    ros::Subscriber mImuSub, mGpsSub, mVelSub;
    ros::Publisher mOdometryPub;
    tf2_ros::TransformBroadcaster mTfBroadcaster;
    tf2_ros::Buffer mTfBuffer;
    tf2_ros::TransformListener mTfListener{mTfBuffer};
    InvariantEKF mEKF;
    TimePoint mLastImuTime;

    InvariantEKF init_EKF();

    void imu_mag_callback(const mrover::ImuAndMag& msg);

    void imu_callback(const sensor_msgs::Imu& msg);

    void gps_callback(const geometry_msgs::PoseWithCovarianceStamped& msg);

    void mag_callback(const sensor_msgs::MagneticField& msg);

    void vel_callback(const geometry_msgs::Twist& msg);

    void publish_odometry();

    void publish_tf();

public:
    int main();

    InvariantEKFNode();
    InvariantEKFNode(const InvariantEKFNode&) = delete;
    InvariantEKFNode& operator=(const InvariantEKFNode&) = delete;

    void run();
};