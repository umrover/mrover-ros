#include <manif/impl/se2/SE2.h>
#include <manif/manif.h>
#include <Eigen/Geometry>

class TerrainParticleFilter {
private:
    Eigen::MatrixXd mTerrainCloud;
    
    // TODO: do this better
    std::vector<manif::SE2d> mParticles;
    std::vector<double> mParticleWeights;
    double mSigmaX, mSigmaTheta;
    double mFootprintX, mFootprintY;

    // TODO: add const and no discard
    void load_terrain_cloud(std::string const& filename);
    void init_particles(manif::SE2d const& initialPose);
    Eigen::Vector3d get_surface_normal(manif::SE2d const& pose);

public:
    TerrainParticleFilter(std::string const& terrainFilename, double sigmaX, double sigmaTheta, double footprintX, double footprintY);   
    void init(manif::SE2d const& initialPose);

    // TODO: add overloads for odometry and IMU pose
    void predict(Eigen::Vector3d const& velCmd, double dt);
    void update(Eigen::Vector3d const& accelMeasurement);
};