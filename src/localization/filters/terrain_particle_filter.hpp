#include <manif/impl/se2/SE2.h>
#include <manif/manif.h>
#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <random>
#include <optional>

struct TerrainMap {
    Eigen::MatrixXd grid;
    Eigen::Vector2d origin; // origin of map in meters relative to grid origin
    double metersPerCell;
};

class TerrainParticleFilter {
private:
    TerrainMap mTerrainMap;
    TerrainMap mNeighborhood;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mNeighborhoodCloud;
     
    // TODO: do this better
    std::vector<manif::SE2d> mParticles;
    std::vector<double> mParticleWeights;
    manif::SE2d mPoseEstimate;
    std::normal_distribution<double> mXDist, mThetaDist, mResamplingXDist, mResamplingThetaDist;
    std::uniform_real_distribution<> mXDistUniform, mYDistUniform, mThetaDistUniform;
    std::default_random_engine mRNG;
    Eigen::Vector2d mFootprint;
    Eigen::Vector3d mVelocity;
    double mRandomInjectionRate;

    static TerrainMap load_terrain_map(const std::string& filename);
    void update_pose_estimate(const std::vector<manif::SE2d>& particles, const std::vector<double>& weights);

public:
    TerrainParticleFilter(const std::string& terrainFilename, double sigmaX, double sigmaTheta, const Eigen::Vector2d& footprint);   
    void init_particles(const manif::SE2d& initialPose, size_t numParticles);
    void init_particles(size_t numParticles);

    // TODO: add overloads for odometry and IMU pose
    void predict(const Eigen::Vector3d& velCmd, double dt);
    void predict(const Eigen::Quaterniond& orientation, const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro, double dt);
    void update(const Eigen::Quaterniond& orientation);

    [[nodiscard]] Eigen::Vector2d idx_to_position(const Eigen::Vector2i& idx) const;
    [[nodiscard]] Eigen::Vector2i position_to_idx(const Eigen::Vector2d& position, bool clampBounds = true) const;

    [[nodiscard]] const manif::SE2d& get_pose_estimate() const;
    [[nodiscard]] const std::vector<manif::SE2d>& get_particles() const;
    [[nodiscard]] const Eigen::MatrixXd& get_terrain_grid();
    [[nodiscard]] pcl::PointCloud<pcl::PointXYZ>::Ptr get_neighborhood();
    [[nodiscard]] std::optional<Eigen::Vector3d> get_surface_normal(const manif::SE2d& pose);
};