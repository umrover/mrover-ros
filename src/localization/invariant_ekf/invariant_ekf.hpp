#include <Eigen/Dense>
#include <manif/manif.h>

using Eigen::Matrix3d, Eigen::Vector3d;
using manif::SE_2_3d, manif::SE_2_3Tangentd;
using Matrix9d = Eigen::Matrix<double, 9, 9>;

class InvariantEKF {
public:
    InvariantEKF(const SE_2_3d& x0, const Matrix9d& P0, const Matrix9d& Q, const Matrix3d& R_gps_default, const Matrix3d& R_accel_default, const Matrix3d& R_mag_default);

    InvariantEKF(const InvariantEKF&) = default;
    InvariantEKF& operator=(const InvariantEKF&) = delete;

    void predict(const Vector3d& accel, const Vector3d& gyro, double dt);

    void update_gps(const Vector3d& observed_gps, const Matrix3d& R_gps);
    void update_gps(const Vector3d& observed_gps);

    void update_accel(const Vector3d& observed_accel, const Matrix3d& R_accel);
    void update_accel(const Vector3d& observed_accel);

    void update_mag(const Vector3d& observed_mag, const Matrix3d& R_mag);
    void update_mag(const Vector3d& observed_mag);

    [[nodiscard]] const SE_2_3d& get_state() const;
    [[nodiscard]] const Matrix9d& get_covariance() const;

private:
    SE_2_3d mX;
    Matrix9d mP, mQ;
    Matrix3d mR_gps, mR_accel, mR_mag;
};