#include <manif/impl/se2/SE2.h>
#include <manif/manif.h>
#include <Eigen/Geometry>
#include <random>

struct TerrainMap {
    Eigen::MatrixXd grid;
    Eigen::Vector2d origin; // origin of map in meters relative to grid origin
    double metersPerCell;
};

class TerrainParticleFilter {
private:
    TerrainMap mTerrainMap;
     
    // TODO: do this better
    std::vector<manif::SE2d> mParticles;
    std::vector<double> mParticleWeights;
    manif::SE2d mPoseEstimate;
    std::normal_distribution<double> mXDist, mThetaDist;
    std::default_random_engine mRNG;
    double mFootprintX, mFootprintY;

    void load_terrain_map(const std::string& filename);
    [[nodiscard]] const Eigen::Vector3d get_surface_normal(const manif::SE2d& pose) const;

public:
    TerrainParticleFilter(const std::string& terrainFilename, double sigmaX, double sigmaTheta, double footprintX, double footprintY);   
    void init_particles(const manif::SE2d& initialPose, int numParticles);

    // TODO: add overloads for odometry and IMU pose
    void predict(const Eigen::Vector3d& velCmd, double dt);
    void update(const Eigen::Vector3d& accelMeasurement);

    [[nodiscard]] const Eigen::Vector2d idx_to_position(const Eigen::Vector2i& idx) const;
    [[nodiscard]] const Eigen::Vector2i position_to_idx(const Eigen::Vector2d& position) const;

    [[nodiscard]] const manif::SE2d& get_pose_estimate() const;
    [[nodiscard]] const std::vector<manif::SE2d>& get_particles() const;
    [[nodiscard]] const Eigen::MatrixXd& get_terrain_grid() const;
};