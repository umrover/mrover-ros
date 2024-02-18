#include "invariant_ekf.hpp"

InvariantEKF::InvariantEKF(SE_2_3d const& x0, Matrix9d const& P0, Matrix9d const& Q, Matrix3d const& R_gps_default, Matrix3d const& R_accel_default, Matrix3d const& R_mag_default)
    : mX(x0), mP(P0), mQ(Q), mR_gps(R_gps_default), mR_accel(R_accel_default), mR_mag(R_mag_default) {}

void InvariantEKF::predict(const R3& accel, const R3& gyro, double dt) {
    // subtract gravity vector from measured acceleration
    // TODO: make static member variable?
    const R3 accel_g(0, 0, g);
    Matrix3d R = mX.rotation();
    R3 accel_body = accel - R.transpose() * accel_g;
    std::cout << "accel_body: " << accel_body << std::endl;

    // use kinematics to compute increments to each state component
    R3 delta_pos = (R.transpose() * mX.linearVelocity() * dt) + (0.5 * accel_body * dt * dt);
    R3 delta_rot = gyro * dt;
    R3 delta_v = accel_body * dt;

    Matrix3d accLinCross = manif::skew(delta_pos);
    R3 gLin = R.transpose() * accel_g * dt;
    Matrix3d gCross = manif::skew(gLin);


    // TODO: pre allocate matrices? (everywhere, not just here)
    SE_2_3d::Tangent u;
    SE_2_3d::Jacobian F, J_x_x, J_x_u, J_u_x;

    J_u_x.setZero();
    J_u_x.block<3, 3>(0, 3) = accLinCross;
    J_u_x.block<3, 3>(0, 6) = Matrix3d::Identity() * dt;
    J_u_x.block<3, 3>(6, 3) = gCross;

    // increment state with right plus operation and compute jacobians
    u << delta_pos, delta_rot, delta_v;
    mX = mX.rplus(u, J_x_x, J_x_u);

    // TODO: where does this come from? chain rule + Ax + Bu?
    // need to set J_u_x
    F = J_x_x + J_x_u * J_u_x;
    mP = F * mP * F.transpose() + J_x_u * mQ * J_x_u.transpose();
}

void InvariantEKF::update(const R3& innovation, Matrix39d const& H, Matrix3d const& R) {
    // innovation covariance
    auto S = (H * mP * H.transpose()) + R;

    // Kalman gain
    auto K = mP * H.transpose() * S.inverse();

    // update state mean (overloaded rplus)
    SE_2_3d::Tangent dx = K * innovation;
    mX += dx;

    // update state covariance
    mP = mP - K * S * K.transpose();
}

void InvariantEKF::update_gps(R3 const& observed_gps, Matrix3d const& R_gps) {
    const R3 origin = R3::Zero();
    Matrix39d J_e_x = Matrix39d::Zero();

    //R(0) + t = t
    R3 predicted_gps = mX.act(origin, J_e_x);
    Matrix39d H = J_e_x;
    update(observed_gps - predicted_gps, H, R_gps);
}

void InvariantEKF::update_gps(R3 const& observed_gps) {
    update_gps(observed_gps, mR_gps);
}

//vel
void InvariantEKF::update_accel(R3 const& observed_accel, Matrix3d const& R_accel) {
    const R3 g(0, 0, 1);
    SO3d R = mX.asSO3();
    SO3d::Jacobian J_Ri_R, J_e_Ri, J_e_R;

    R3 predicted_accel = R.inverse(J_Ri_R).act(g, J_e_Ri);
    J_e_R = J_e_Ri * J_Ri_R;

    // final jacobian
    Matrix39d J_e_x;
    J_e_x.setZero();
    J_e_x.block(0, 0, 3, 3) = J_e_R;
    Matrix39d H = J_e_x;
    update(observed_accel - predicted_accel, H, R_accel);
}

void InvariantEKF::update_accel(R3 const& observed_accel) {
    update_accel(observed_accel, mR_accel);
}


//orientation
void InvariantEKF::update_mag(R3 const& observed_mag, Matrix3d const& R_mag) {
    const R3 north{0, 1, 0};
    SO3d R = mX.asSO3();
    SO3d::Jacobian J_Ri_R, J_e_Ri, J_e_R;

    R3 predicted_mag = R.inverse(J_Ri_R).act(north, J_e_Ri);
    J_e_R = J_e_Ri * J_Ri_R;

    //final jacobian
    //source: https://arxiv.org/pdf/2007.14097.pdf
    Matrix39d J_e_x;
    J_e_x.setZero();
    J_e_x.block(0, 0, 3, 3) = J_e_R;
    Matrix39d H = J_e_x;
    update(observed_mag - predicted_mag, H, R_mag);
}

void InvariantEKF::update_mag(R3 const& observed_mag) {
    update_mag(observed_mag, mR_mag);
}

[[nodiscard]] SE_2_3d const& InvariantEKF::get_state() const {
    return mX;
}

[[nodiscard]] Matrix9d const& InvariantEKF::get_covariance() const {
    return mP;
}