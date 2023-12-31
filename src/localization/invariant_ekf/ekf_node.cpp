#include "ekf_node.hpp"

InvariantEKFNode::InvariantEKFNode() {
    // set initial position to zero and initial covariant to very high number
    auto x0 = SE_2_3d::Identity();
    auto P0 = Matrix9d::Identity() * 1e6;

    // load covariance matrices from rosparam
    Matrix9d Q;
    Matrix3d R_gps, R_accel, R_mag;
    std::vector<double> Q_vec, R_gps_vec, R_accel_vec, R_mag_vec;
    if (!mPnh.getParam("process_noise_covariance", Q_vec)) {
        throw std::runtime_error("Failed to load process_noise_covariance from rosparam");
    }
    if (!mPnh.getParam("gps_covariance", R_gps_vec)) {
        throw std::runtime_error("Failed to load gps_covariance from rosparam");
    }
    if (!mPnh.getParam("accel_covariance", R_accel_vec)) {
        throw std::runtime_error("Failed to load accel_covariance from rosparam");
    }
    if (!mPnh.getParam("mag_covariance", R_mag_vec)) {
        throw std::runtime_error("Failed to load mag_covariance from rosparam");
    }

    // set up subscribers and publishers
}