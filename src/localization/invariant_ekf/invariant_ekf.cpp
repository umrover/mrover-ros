#include "invariant_ekf.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <manif/impl/se_2_3/SE_2_3.h>
#include <manif/impl/se_2_3/SE_2_3Tangent.h>
#include <manif/impl/so3/SO3.h>

InvariantEKF::InvariantEKF(const SE_2_3d& x0, const Matrix9d& P0, const Matrix9d& Q, const Matrix3d& R_gps_default, const Matrix3d& R_accel_default, const Matrix3d& R_mag_default)
    : mX(x0), mP(P0), mQ(Q), mR_gps(R_gps_default), mR_accel(R_accel_default), mR_mag(R_mag_default) {}

void InvariantEKF::predict(SE_2_3Tangentd const& u, double dt) {
    //use kinematic equations to predict pose based on accel and gyro

    //idk if intended to directly apply the u passed in but writing the formula out
    Eigen::Vector3d g(0, 0, -9.81);
    Matrix3d R = mX.rotation(); //rotation matrix

    Vector3d lin_acc = u.lin2();
    Vector3d ang_vel = u.ang();

    Vector3d acc_adj = lin_acc + R.transpose() * g;

    Vector3d delta_pos = (R.transpose() * mX.linearVelocity() * dt) + (0.5 * acc_adj * dt * dt);
    Vector3d delta_rot = ang_vel * dt;
    Vector3d delta_v = acc_adj * dt;

    SE_2_3Tangentd u_est;
    u_est << delta_pos, delta_rot, delta_v;

    manif::SE_2_3d::Jacobian F;
    manif::SE_2_3d::Jacobian J_x_x;
    manif::SE_2_3d::Jacobian J_x_u;
    manif::SE_2_3d::Jacobian J_u_x;

    mX = mX.plus(u_est, J_x_x, J_x_u);

    //don't understand J_u_x
    F = J_x_x + (J_x_u * J_u_x); //from example
    //does Q stay the same throughout? Sola paper uses jacobians here and the other says Q = Cov(wn), same thing?
    mP = F * mP * F.transpose() + mQ;
}

void InvariantEKF::update_gps(Eigen::Vector3d const& z, Matrix3d const& R_gps) {
    //Yk = h_x + noise
    Vector3d Yk; //gps reading
    Vector3d h_x = mX.translation(); //what is the jacobian of this

    auto test = mX.rotation(); //rotation matrix
    test = test.inverse(); //inverse of matrix

    Eigen::Vector3d b(0, 0, 0);
    b = b * test; //rotate vector by the inverse

    manif::SE_2_3d::Jacobian J_h_x;

    //this gives correct jacobian? does the velocity mess it up? rotation gets cancelled out so left with translation
    //this seems like a stupid way to get the jacobian
    Vector3d e = mX.act(b, J_h_x); 

    auto H = J_h_x;

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

void InvariantEKF::update_gps(Eigen::Vector3d const& z) {
    update_gps(z, mR_gps);
}

void InvariantEKF::update_accel(Eigen::Vector3d const& z, Matrix3d const& R_accel) {}

void InvariantEKF::update_accel(Eigen::Vector3d const& z) {
    update_accel(z, mR_accel);
}

void InvariantEKF::update_mag(Eigen::Vector3d const& z, Matrix3d const& R_mag) {}

void InvariantEKF::update_mag(Eigen::Vector3d const& z) {
    update_mag(z, mR_mag);
}

[[nodiscard]] SE_2_3d const& InvariantEKF::get_state() const {
    return mX;
}

[[nodiscard]] Matrix9d const& InvariantEKF::get_covariance() const {
    return mP;
}