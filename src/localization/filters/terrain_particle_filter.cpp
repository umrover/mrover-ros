#include "terrain_particle_filter.hpp"
#include <Eigen/Core>
#include <cstddef>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <random>

TerrainParticleFilter::TerrainParticleFilter(const std::string& terrainFilename, double sigmaX, double sigmaTheta, const Eigen::Vector2d& footprint) : mXDist(0, sigmaX), mThetaDist(0, sigmaTheta), mFootprint(footprint) {
    assert(footprint.x() > 0 && footprint.y() > 0);
    assert(sigmaX >= 0 && sigmaTheta >= 0);
    load_terrain_map(terrainFilename);
}

void TerrainParticleFilter::load_terrain_map(const std::string& filename) {
    cv::Mat terrainImage = cv::imread(filename, cv::IMREAD_LOAD_GDAL);
    std::cout << terrainImage.depth() << " " << terrainImage.channels() << std::endl;
    if (terrainImage.empty()) throw std::runtime_error("Could not open terrain file");

    cv::rotate(terrainImage, terrainImage, cv::ROTATE_90_CLOCKWISE);
    double width = 50.0, height = 50.0;
    mTerrainMap.origin = Eigen::Vector2d(width / 2.0, height / 2.0);
    mTerrainMap.metersPerCell = width / terrainImage.cols;
    mTerrainMap.grid = Eigen::MatrixXd(terrainImage.rows, terrainImage.cols);
    for (int i = 0; i < terrainImage.rows; i++) {
        for (int j = 0; j < terrainImage.cols; j++) {
            mTerrainMap.grid(i, j) = terrainImage.at<double>(i, j);
            // std::cout <<terrainImage.at<double>(i, j) << " ";
        }
    }
    // cv::imshow("Terrain", terrainImage);
    // cv::waitKey(0);
}

const Eigen::Vector2d TerrainParticleFilter::idx_to_position(const Eigen::Vector2i& idx) const {
    return idx.cast<double>() * mTerrainMap.metersPerCell - mTerrainMap.origin;
}

const Eigen::Vector2i TerrainParticleFilter::position_to_idx(const Eigen::Vector2d& position, bool clampBounds = true) const {
    Eigen::Vector2i idx = ((position + mTerrainMap.origin) / mTerrainMap.metersPerCell).cast<int>();
    if (clampBounds) {
        // TODO: use vector of longs for idx?
        idx.x() = std::clamp(idx.x(), 0, static_cast<int>(mTerrainMap.grid.rows() - 1));
        idx.y() = std::clamp(idx.y(), 0, static_cast<int>(mTerrainMap.grid.cols() - 1));
    }
    return idx;
}

void TerrainParticleFilter::init_particles(const manif::SE2d& initialPose, int numParticles) {
    mParticles = std::vector<manif::SE2d>(numParticles, initialPose);
}

// TODO: make it const
const Eigen::Vector3d TerrainParticleFilter::get_surface_normal(manif::SE2d const& pose) {
    Eigen::Vector2i minCorner = position_to_idx(pose.translation() - mFootprint / 2.0);
    Eigen::Vector2i maxCorner = position_to_idx(pose.translation() + mFootprint / 2.0);
    Eigen::Vector2i footprintCells = maxCorner - minCorner;
    Eigen::MatrixXd neighborhood = mTerrainMap.grid.block(minCorner.x(), minCorner.y(),
                                                         footprintCells.x(), footprintCells.y());
    if(neighborhood.rows() == 0 || neighborhood.cols() == 0)
        throw std::runtime_error("pose out of bounds of terrain map");
    // Eigen::MatrixXd neighborhood = mTerrainMap.grid(Eigen::seq(minCorner.x(), maxCorner.x()), Eigen::seq(minCorner.y(), maxCorner.y()));

    mNeighborhood = mTerrainMap;
    mNeighborhood.grid = Eigen::MatrixXd::Zero(mTerrainMap.grid.rows(), mTerrainMap.grid.cols());
    mNeighborhood.grid.block(minCorner.x(), minCorner.y(),
                             footprintCells.x(), footprintCells.y()) = neighborhood;
    return Eigen::Vector3d();
}

void TerrainParticleFilter::predict(const Eigen::Vector3d& velCmd, double dt) {
    manif::SE2Tangentd increment;
    for (auto& particle: mParticles) {
        Eigen::Vector3d velNoise(mXDist(mRNG), 0, mThetaDist(mRNG));
        increment = (velCmd + velNoise) * dt;
        particle = particle.rplus(increment);
    }
}

void TerrainParticleFilter::update(const Eigen::Vector3d& accelMeasurement) {
}

const manif::SE2d& TerrainParticleFilter::get_pose_estimate() const {
    // return mPoseEstimate;
    return mParticles[0];
}

const std::vector<manif::SE2d>& TerrainParticleFilter::get_particles() const {
    return mParticles;
}

// TODO: make it const
const Eigen::MatrixXd& TerrainParticleFilter::get_terrain_grid() {
    auto v = get_surface_normal(mParticles[0]);
    // std::cout << mNeighborhood.grid << std::endl;
    return mNeighborhood.grid;
    // return mTerrainMap.grid;
}
