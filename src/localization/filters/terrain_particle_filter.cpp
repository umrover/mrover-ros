#include "terrain_particle_filter.hpp"

TerrainParticleFilter::TerrainParticleFilter(std::string const& terrainFilename, double sigmaX, double sigmaTheta, double footprintX, double footprintY) : mSigmaX(sigmaX), mSigmaTheta(sigmaTheta), mFootprintX(footprintX), mFootprintY(footprintY) {
    load_terrain_cloud(terrainFilename);
}

void TerrainParticleFilter::load_terrain_cloud(std::string const& filename) {
}

void TerrainParticleFilter::init_particles(manif::SE2d const& initialPose) {
}

Eigen::Vector3d TerrainParticleFilter::get_surface_normal(manif::SE2d const& pose) {
}

void TerrainParticleFilter::init(manif::SE2d const& initialPose) {
}

void TerrainParticleFilter::predict(Eigen::Vector3d const& velCmd, double dt) {
}

void TerrainParticleFilter::update(Eigen::Vector3d const& accelMeasurement) {
}
