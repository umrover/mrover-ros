#include <manif/impl/se2/SE2.h>
#include <manif/manif.h>
#include <Eigen/Geometry>
#include <random>

class TerrainParticleFilter {
private:
    Eigen::MatrixXd mTerrainCloud;
    
    // TODO: do this better
    std::vector<manif::SE2d> mParticles;
    std::vector<double> mParticleWeights;
    manif::SE2d mPoseEstimate;
    std::normal_distribution<double> mXDist, mThetaDist;
    std::default_random_engine mRNG;
    double mFootprintX, mFootprintY;

    void load_terrain_cloud(const std::string& filename);
    [[nodiscard]] const Eigen::Vector3d get_surface_normal(const manif::SE2d& pose) const;

public:
    TerrainParticleFilter(const std::string& terrainFilename, double sigmaX, double sigmaTheta, double footprintX, double footprintY);   
    void init_particles(const manif::SE2d& initialPose, int numParticles);

    // TODO: add overloads for odometry and IMU pose
    void predict(const Eigen::Vector3d& velCmd, double dt);
    void update(const Eigen::Vector3d& accelMeasurement);

    [[nodiscard]] const manif::SE2d& get_pose_estimate() const;
    [[nodiscard]] const std::vector<manif::SE2d>& get_particles() const;
};