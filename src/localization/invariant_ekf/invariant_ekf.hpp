#include <Eigen/Dense>
#include <manif/manif.h>

using Eigen::Matrix3d, Eigen::Vector3d;
using manif::SE_2_3d, manif::SE_2_3Tangentd;
using Matrix9d = Eigen::Matrix<double, 9, 9>;

class InvariantEKF {
public:
    InvariantEKF(const SE_2_3d& x0, const Matrix9d& P0, const Matrix9d& Q, const Matrix9d& R_gps_default, const Matrix9d& R_accel_default, const Matrix9d& R_mag_default);

    InvariantEKF(const InvariantEKF&) = delete;
    InvariantEKF& operator=(const InvariantEKF&) = delete;

    void predict(SE_2_3Tangentd const& u, double dt);

    void update_gps(Eigen::Vector3d const& z, Matrix3d const& R_gps);
    void update_gps(Eigen::Vector3d const& z);

    void update_accel(Eigen::Vector3d const& z, Matrix3d const& R_accel);
    void update_accel(Eigen::Vector3d const& z);

    void update_mag(Eigen::Vector3d const& z, Matrix3d const& R_mag);
    void update_mag(Eigen::Vector3d const& z);

    [[nodiscard]] SE_2_3d const& get_state() const;
    [[nodiscard]] Matrix9d const& get_covariance() const;

private:
    SE_2_3d mX;
    Matrix9d mP, mQ;
    Matrix3d mR_gps, mR_accel, mR_mag;
};