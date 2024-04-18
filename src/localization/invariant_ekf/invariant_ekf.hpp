#include "pch.hpp"

using Eigen::Matrix3d, Eigen::Vector3d;
using manif::SE_2_3d;
using Matrix9d = Eigen::Matrix<double, 9, 9>;
using Matrix39d = Eigen::Matrix<double, 3, 9>;
using Matrix93d = Eigen::Matrix<double, 9, 3>;

constexpr double g = 9.81;
static const R3 accel_g(0, 0, g);

class InvariantEKF {
public:
    InvariantEKF(const SE_2_3d& x0, const Matrix9d& P0, const Matrix9d& Q, const Matrix3d& R_gps_default, const Matrix3d& R_accel_default, const Matrix3d& R_mag_default, const Matrix3d& R_vel_default);

    InvariantEKF(const InvariantEKF&) = default;
    InvariantEKF& operator=(const InvariantEKF&) = delete;

    void predict(const R3& accel, const R3& gyro, double dt);
    void update(const R3& innovation, const Matrix39d& H, const Matrix3d& R);


    void update_gps(const R3& observed_gps, const Matrix3d& R_gps);
    void update_gps(const R3& observed_gps);

    void update_accel(const R3& observed_accel, const Matrix3d& R_accel);
    void update_accel(const R3& observed_accel);

    void update_mag(const R3& observed_mag, const Matrix3d& R_mag);
    void update_mag(const R3& observed_mag);

    void update_vel(const R3& observed_vel, const Matrix3d& R_vel);
    void update_vel(const R3& observed_vel);


    [[nodiscard]] const SE_2_3d& get_state() const;
    [[nodiscard]] const Matrix9d& get_covariance() const;

private:
    SE_2_3d mX;
    Matrix9d mP, mQ;
    Matrix3d mR_gps, mR_accel, mR_mag, mR_vel;
};