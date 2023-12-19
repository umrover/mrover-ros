#include "invariant_ekf.hpp"

InvariantEKF::InvariantEKF(const SE_2_3d& x0, const Matrix9d& P0, const Matrix9d& Q, const Matrix9d& R_gps_default, const Matrix9d& R_accel_default, const Matrix9d& R_mag_default)
    : mX(x0), mP(P0), mQ(Q), mR_gps(R_gps_default), mR_accel(R_accel_default), mR_mag(R_mag_default) {}

void InvariantEKF::predict(SE_2_3Tangentd const& u, double dt) {}

void InvariantEKF::update_gps(Eigen::Vector3d const& z, Matrix3d const& R_gps) {}

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