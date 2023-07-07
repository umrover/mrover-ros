#include "terrain_particle_filter.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cstddef>
#include <random>
#include <iostream>

TerrainParticleFilter::TerrainParticleFilter(const std::string& terrainFilename, double sigmaX, double sigmaTheta, double footprintX, double footprintY) : mXDist(0, sigmaX), mThetaDist(0, sigmaTheta), mFootprintX(footprintX), mFootprintY(footprintY) {
    load_terrain_cloud(terrainFilename);
}

void TerrainParticleFilter::load_terrain_cloud(const std::string& filename) {
    cv::Mat terrainImage = cv::imread(filename, cv::IMREAD_LOAD_GDAL);
    std::cout << terrainImage.depth() << " " << terrainImage.channels() << std::endl;
    if (terrainImage.empty()) throw std::runtime_error("Could not open terrain file");
    
    cv::rotate(terrainImage, terrainImage, cv::ROTATE_90_COUNTERCLOCKWISE);
    mTerrainCloud = Eigen::MatrixXd(terrainImage.rows, terrainImage.cols);
    for (int i = 0; i < terrainImage.rows; i++) {
        for (int j = 0; j < terrainImage.cols; j++) {
            mTerrainCloud(i, j) = terrainImage.at<double>(i, j);
            // std::cout <<terrainImage.at<double>(i, j) << " ";
        }
    }
    // cv::imshow("Terrain", terrainImage);
    // cv::waitKey(0);
}

void TerrainParticleFilter::init_particles(const manif::SE2d& initialPose, int numParticles) {
    mParticles = std::vector<manif::SE2d>(numParticles, initialPose);
}

const Eigen::Vector3d TerrainParticleFilter::get_surface_normal(manif::SE2d const& pose) const {
    return Eigen::Vector3d();
}

void TerrainParticleFilter::predict(const Eigen::Vector3d& velCmd, double dt) {
    manif::SE2Tangentd increment;
    for (auto& particle : mParticles) {
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

const Eigen::MatrixXd& TerrainParticleFilter::get_terrain_cloud() const {
    return mTerrainCloud;
}
