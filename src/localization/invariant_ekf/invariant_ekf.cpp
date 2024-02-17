#include "invariant_ekf.hpp"

InvariantEKF::InvariantEKF(const SE_2_3d& x0, const Matrix9d& P0, const Matrix9d& Q, const Matrix3d& R_gps_default, const Matrix3d& R_accel_default, const Matrix3d& R_mag_default)
    : mX(x0), mP(P0), mQ(Q), mR_gps(R_gps_default), mR_accel(R_accel_default), mR_mag(R_mag_default) {}

void InvariantEKF::predict(const Vector3d& accel, const Vector3d& gyro, double dt) {
    // subtract gravity vector from measured acceleration
    // TODO: make static member variable?
    const R3 g(0, 0, -9.81);
    Matrix3d R = mX.rotation();
    Vector3d accel_body = accel + R.transpose() * g;

    // use kinematics to compute increments to each state component
    Vector3d delta_pos = (R.transpose() * mX.linearVelocity() * dt) + (0.5 * accel_body * dt * dt);
    Vector3d delta_rot = gyro * dt;
    Vector3d delta_v = accel_body * dt;

    Eigen::Matrix3d accLinCross = manif::skew(delta_pos);
    Eigen::Vector3d gLin = R.transpose()*g*dt;
    Eigen::Matrix3d gCross = manif::skew(gLin);


    SE_2_3Tangentd u;
    manif::SE_2_3d::Jacobian F, J_x_x, J_x_u, J_u_x;

    J_u_x.setZero();
    J_u_x.block<3, 3>(0, 3) = accLinCross;
    J_u_x.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity()*dt;
    J_u_x.block<3, 3>(6, 3) = gCross;

    // increment state with right plus operation and compute jacobians
    u << delta_pos, delta_rot, delta_v;
    mX = mX.rplus(u, J_x_x, J_x_u);

    // TODO: where does this come from? chain rule + Ax + Bu?
    // need to set J_u_x
    F = J_x_x + J_x_u * J_u_x;
    mP = F * mP * F.transpose() + J_x_u * mQ * J_x_u.transpose();
}

//pos
void InvariantEKF::update_gps(Eigen::Vector3d const& observed_gps, Matrix3d const& R_gps) {
    Eigen::Vector3d zero = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 3, 9> J_e_x;
    J_e_x.setZero();

    //R(0) + t = t
    Vector3d expected_gps = mX.act(zero, J_e_x);
    Eigen::Matrix<double, 3, 9> H = J_e_x;

    //innovation
    auto z = observed_gps - expected_gps;

    //innovation cov
    auto S_n = (H * mP * H.transpose()) + R_gps;

    //kalman gain
    auto K_n = mP * H.transpose() * S_n.inverse();

    //observed error
    manif::SE_2_3Tangentd dx = K_n * z;

    //state update
    mX = mX.plus(dx);

    //cov update
    mP = mP - K_n * S_n * K_n.transpose();
}

void InvariantEKF::update_gps(Eigen::Vector3d const& observed_gps) {
    update_gps(observed_gps, mR_gps);
}

//vel
void InvariantEKF::update_accel(Eigen::Vector3d const& observed_accel, Matrix3d const& R_accel) {
    const Vector3d g(0, 0, 1);
    SO3d R = mX.asSO3();
    SO3d::Jacobian J_Ri_R, J_e_Ri, J_e_R;

    Vector3d predicted_accel = R.inverse(J_Ri_R).act(g, J_e_Ri);
    J_e_R = J_e_Ri * J_Ri_R;

    // final jacobian
    Eigen::Matrix<double, 3, 9> J_e_x;
    J_e_x.setZero();
    J_e_x.block(0, 0, 3, 3) = J_e_R;
    Eigen::Matrix<double, 3, 9> H = J_e_x;

    //innovation
    auto z = observed_accel - predicted_accel;

    //innovation cov
    auto S_n = (H * mP * H.transpose()) + R_accel;

    //kalman gain
    auto K_n = mP * H.transpose() * S_n.inverse();

    //observed error
    manif::SE_2_3Tangentd dx = K_n * z;

    //state update
    mX = mX.plus(dx);

    //cov update
    mP = mP - K_n * S_n * K_n.transpose();
}

void InvariantEKF::update_accel(Eigen::Vector3d const& observed_accel) {
    update_accel(observed_accel, mR_accel);
}


//orientation
void InvariantEKF::update_mag(Eigen::Vector3d const& observed_mag, Matrix3d const& R_mag) {
    const Vector3d north(0, 1, 0);
    SO3d R = mX.asSO3();
    SO3d::Jacobian J_Ri_R, J_e_Ri, J_e_R;

    Vector3d e = R.inverse(J_Ri_R).act(north, J_e_Ri);
    J_e_R = J_e_Ri * J_Ri_R;

    //final jacobian
    //source: https://arxiv.org/pdf/2007.14097.pdf
    Eigen::Matrix<double, 3, 9> J_e_x;
    J_e_x.setZero();
    J_e_x.block(0, 0, 3, 3) = J_e_R;
    Eigen::Matrix<double, 3, 9> H = J_e_x;

    //innovation
    auto z = observed_mag - e;

    //innovation cov
    auto S_n = (H * mP * H.transpose()) + R_mag;

    //kalman gain
    auto K_n = mP * H.transpose() * S_n.inverse();

    //observed error
    manif::SE_2_3Tangentd dx = K_n * z;

    //state update
    mX = mX.plus(dx);

    //cov update
    mP = mP - K_n * S_n * K_n.transpose();
}

void InvariantEKF::update_mag(Eigen::Vector3d const& observed_mag) {
    update_mag(observed_mag, mR_mag);
}

[[nodiscard]] SE_2_3d const& InvariantEKF::get_state() const {
    return mX;
}

[[nodiscard]] Matrix9d const& InvariantEKF::get_covariance() const {
    return mP;
}