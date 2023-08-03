#include "terrain_particle_filter.hpp"
#include <Eigen/Core>
#include <cstddef>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/features/normal_3d.h>

TerrainParticleFilter::TerrainParticleFilter(const std::string& terrainFilename, double sigmaX, double sigmaTheta, const Eigen::Vector2d& footprint) : mXDist(0, sigmaX), mThetaDist(0, sigmaTheta), mFootprint(footprint) {
    assert(footprint.x() > 0 && footprint.y() > 0);
    assert(sigmaX >= 0 && sigmaTheta >= 0);
    load_terrain_map(terrainFilename);
}

void TerrainParticleFilter::load_terrain_map(const std::string& filename) {
    cv::Mat terrainImage = cv::imread(filename, cv::IMREAD_LOAD_GDAL);
    // std::cout << terrainImage.depth() << " " << terrainImage.channels() << std::endl;
    if (terrainImage.empty()) throw std::runtime_error("Could not open terrain file");

    cv::rotate(terrainImage, terrainImage, cv::ROTATE_90_CLOCKWISE);
    double width = 50.0, height = 50.0;
    mTerrainMap.origin = Eigen::Vector2d(width / 2.0, height / 2.0);
    mTerrainMap.metersPerCell = width / terrainImage.cols;
    // std::cout << "Meters per cell: " << mTerrainMap.metersPerCell << std::endl;
    // std::cout << "Origin: " << mTerrainMap.origin.transpose() << std::endl;
    mTerrainMap.grid = Eigen::MatrixXd(terrainImage.rows, terrainImage.cols);
    for (int i = 0; i < terrainImage.rows; i++) {
        for (int j = 0; j < terrainImage.cols; j++) {
            mTerrainMap.grid(i, j) = terrainImage.at<double>(j, i);
            // std::cout <<terrainImage.at<double>(i, j) << " ";
        }
    }
    // cv::imshow("Terrain", terrainImage);
    // cv::waitKey(0);
}

Eigen::Vector2d TerrainParticleFilter::idx_to_position(const Eigen::Vector2i& idx) const {
    // TODO: make better choices on use of array vs matrix
    // swap i and j to convert from row-major to column-major
    Eigen::Vector2i temp_idx = idx;
    std::swap(temp_idx.x(), temp_idx.y());

    // offset by half a cell to put point at center of the cell
    return (temp_idx.cast<double>().array() + 0.5).matrix() * mTerrainMap.metersPerCell - mTerrainMap.origin;
}

Eigen::Vector2i TerrainParticleFilter::position_to_idx(const Eigen::Vector2d& position, bool clampBounds) const {
    Eigen::Vector2i idx = ((position + mTerrainMap.origin) / mTerrainMap.metersPerCell).cast<int>();
    std::swap(idx.x(), idx.y());
    if (clampBounds) {
        // TODO: use vector of longs for idx?
        idx.x() = std::clamp(idx.x(), 0, static_cast<int>(mTerrainMap.grid.cols() - 1));
        idx.y() = std::clamp(idx.y(), 0, static_cast<int>(mTerrainMap.grid.rows() - 1));
    }
    return idx;
}

void TerrainParticleFilter::init_particles(const manif::SE2d& initialPose, size_t numParticles) {
    mParticles = std::vector<manif::SE2d>(numParticles, initialPose);
}

void TerrainParticleFilter::init_particles(size_t numParticles) {
    std::uniform_real_distribution<double> xDist(0, mTerrainMap.grid.cols() * mTerrainMap.metersPerCell);
    std::uniform_real_distribution<double> yDist(0, mTerrainMap.grid.rows() * mTerrainMap.metersPerCell);
    std::uniform_real_distribution<double> thetaDist(-M_PI, M_PI);
    mParticles.resize(numParticles);
    for (size_t i = 0; i < numParticles; i++) {
        Eigen::Vector2d position(xDist(mRNG), yDist(mRNG));
        position -= mTerrainMap.origin;
        // mParticles[i] = manif::SE2d(position.x(), position.y(), thetaDist(mRNG));
        mParticles[i] = manif::SE2d(position.x(), position.y(), thetaDist(mRNG));
        // mParticles[i] = manif::SE2d::Random();
    }
}

// TODO: make it const
Eigen::Vector3d TerrainParticleFilter::get_surface_normal(manif::SE2d const& pose) {
    Eigen::Vector2i minCorner = position_to_idx(pose.translation() - mFootprint / 2.0);
    Eigen::Vector2i maxCorner = position_to_idx(pose.translation() + mFootprint / 2.0);
    Eigen::Vector2i footprintCells = maxCorner - minCorner;
    // TODO: index with seq if eigen 3.4 is available
    Eigen::MatrixXd neighborhood = mTerrainMap.grid.block(minCorner.x(), minCorner.y(),
                                                         footprintCells.x(), footprintCells.y());
    if(neighborhood.rows() == 0 || neighborhood.cols() == 0) {
        std::stringstream ss;
        ss << "pose out of bounds of terrain map: " << pose.translation().transpose();
        throw std::runtime_error(ss.str());
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr neighborhood_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < neighborhood.rows(); i++) {
        for (size_t j = 0; j < neighborhood.cols(); j++) {
            if (neighborhood(i, j) > 0) {
                Eigen::Vector2i idx = minCorner + Eigen::Vector2i(i, j);
                Eigen::Vector2d position = idx_to_position(idx);
                neighborhood_cloud->push_back(pcl::PointXYZ(position.x(), position.y(), neighborhood(i, j)));
            }
        }
    }
    Eigen::Vector4f plane_parameters;
    float curvature;
    pcl::computePointNormal(*neighborhood_cloud, plane_parameters, curvature);
    Eigen::Vector3d normal(plane_parameters.x(), plane_parameters.y(), plane_parameters.z());
    // std::cout << "Normal: " << normal.transpose() << std::endl;

    // DEBUG
    neighborhood_cloud->width = neighborhood_cloud->size();
    neighborhood_cloud->height = 1;
    neighborhood_cloud->is_dense = true;
    mNeighborhoodCloud = neighborhood_cloud;
    mNeighborhood = mTerrainMap;
    mNeighborhood.grid = Eigen::MatrixXd::Zero(mTerrainMap.grid.rows(), mTerrainMap.grid.cols());
    mNeighborhood.grid.block(minCorner.x(), minCorner.y(),
                             footprintCells.x(), footprintCells.y()) = neighborhood;
    return normal;
}

void TerrainParticleFilter::update_pose_estimate(const std::vector<manif::SE2d>& particles, const std::vector<double>& weights) {
    size_t maxIdx = std::distance(weights.begin(), std::max_element(weights.begin(), weights.end()));
    mPoseEstimate = particles[maxIdx];
}

void TerrainParticleFilter::predict(const Eigen::Vector3d& velCmd, double dt) {
    manif::SE2Tangentd increment;
    for (auto& particle: mParticles) {
        Eigen::Vector3d velNoise(mXDist(mRNG), 0, mThetaDist(mRNG));
        increment = (velCmd + velNoise) * dt;
        particle = particle.rplus(increment);
    }
}

void TerrainParticleFilter::predict(const Eigen::Quaterniond& orientation, const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro, double dt) {
    // TODO: track velocity to increase accuracy
    // remove gravity vector
    Eigen::Vector3d accelWorld = orientation * accel;
    accelWorld.z() -= 9.81;
    Eigen::Vector3d accelBody = orientation.inverse() * accelWorld;
    std::cout << "Accel: " << accelBody.transpose() << std::endl;

    Eigen::Vector3d velCmd(mVelocity.x() + 0.5 * accelBody.x() * dt, 0, gyro.z());
    predict(velCmd, dt);
    mVelocity += accelBody * dt;
    // for (auto& particle : mParticles) {
    // // only consider x acceleration and z angular velocity
    // // TODO: use angular velocity in world frame
    // double deltaX = (accelBody.x() + mXDist(mRNG)) * dt * dt / 2.0;
    // double deltaTheta = (gyro.z() + mThetaDist(mRNG)) * dt;
    // manif::SE2Tangentd increment(deltaX, 0, deltaTheta);
    // particle = particle.rplus(increment);
    // }
}

void TerrainParticleFilter::update(const Eigen::Quaterniond& orientation) {
    // compute weights
    Eigen::VectorXd weights(mParticles.size());
    for (size_t i = 0; i < mParticles.size(); i++) {
        Eigen::Vector3d normal = get_surface_normal(mParticles[i]);
        Eigen::Vector3d zAxis = orientation * Eigen::Vector3d::UnitZ();
        double weight = std::clamp(normal.dot(zAxis), 0.0, 1.0);
        weights[i] = weight;
    }
    weights /= weights.sum();
    std::vector<double> weights_vec(weights.data(), weights.data() + weights.size());
    update_pose_estimate(mParticles, weights_vec);

    // resample particles
    std::discrete_distribution<int> distribution(weights_vec.begin(), weights_vec.end());
    std::vector<manif::SE2d> newParticles(mParticles.size());
    for (int i = 0; i < mParticles.size(); i++) {
        newParticles[i] = mParticles[distribution(mRNG)];
    }
    mParticles = newParticles;
}

const manif::SE2d& TerrainParticleFilter::get_pose_estimate() const {
    return mPoseEstimate;
    // return mParticles[0];
}

const std::vector<manif::SE2d>& TerrainParticleFilter::get_particles() const {
    return mParticles;
}

// TODO: make it const
const Eigen::MatrixXd& TerrainParticleFilter::get_terrain_grid() {
    // auto v = get_surface_normal(mParticles[0]);
    // std::cout << mNeighborhood.grid << std::endl;
    // return mNeighborhood.grid;
    return mTerrainMap.grid;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr TerrainParticleFilter::get_neighborhood() {
    return mNeighborhoodCloud;
}
