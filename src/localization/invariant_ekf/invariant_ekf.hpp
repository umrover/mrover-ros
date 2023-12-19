#include <Eigen/Dense>
#include <manif/manif.h>

using manif::SE_2_3d;
using manif::SE_2_3Tangentd;
using Matrix9d = Eigen::Matrix<double, 9, 9>;

class InvariantEKF {
public:
    InvariantEKF(SE_2_3d const& x0, Matrix9d const& P0, Matrix9d Q, Matrix9d R_gps_default);

    void predict(SE_2_3Tangentd const& u, double dt);

    void update_gps(Eigen::Vector3d const& z, Matrix9d const& R_gps);

private:
    SE_2_3d mX;
    Matrix9d mP, mQ, mR_gps;
};