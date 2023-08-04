#include "terrain_particle_filter.hpp"
#include <Eigen/Core>
#include <cstddef>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/features/normal_3d.h>

// TODO: make new variables arguments to the constructor
TerrainParticleFilter::TerrainParticleFilter(const std::string& terrainFilename, double sigmaX, double sigmaTheta, const Eigen::Vector2d& footprint) : mTerrainMap(load_terrain_map(terrainFilename)), mXDist(0, sigmaX), mThetaDist(0, sigmaTheta), mResamplingXDist(0, 0.01), mResamplingThetaDist(0, 0.01), mFootprint(footprint), mVelocity(0, 0, 0), mRandomInjectionRate(0.2) {
    assert(footprint.x() > 0 && footprint.y() > 0);
    assert(sigmaX >= 0 && sigmaTheta >= 0);
    Eigen::Vector2d minCorner = idx_to_position(Eigen::Vector2i(0, 0));
    Eigen::Vector2d maxCorner = idx_to_position(Eigen::Vector2i(mTerrainMap.grid.cols() - 1, mTerrainMap.grid.rows() - 1));
    mXDistUniform = std::uniform_real_distribution<>(minCorner.x(), maxCorner.x());
    mYDistUniform = std::uniform_real_distribution<>(minCorner.y(), maxCorner.y());
    mThetaDistUniform = std::uniform_real_distribution<>(-M_PI, M_PI);
}

TerrainMap TerrainParticleFilter::load_terrain_map(const std::string& filename) {
    cv::Mat terrainImage = cv::imread(filename, cv::IMREAD_LOAD_GDAL);
    // std::cout << terrainImage.depth() << " " << terrainImage.channels() << std::endl;
    if (terrainImage.empty()) throw std::runtime_error("Could not open terrain file");

    TerrainMap map;
    cv::rotate(terrainImage, terrainImage, cv::ROTATE_90_CLOCKWISE);
    double width = 50.0, height = 50.0;
    map.origin = Eigen::Vector2d(width / 2.0, height / 2.0);
    map.metersPerCell = width / terrainImage.cols;
    // std::cout << "Meters per cell: " << mTerrainMap.metersPerCell << std::endl;
    // std::cout << "Origin: " << mTerrainMap.origin.transpose() << std::endl;
    map.grid = Eigen::MatrixXd(terrainImage.rows, terrainImage.cols);
    for (int i = 0; i < terrainImage.rows; i++) {
        for (int j = 0; j < terrainImage.cols; j++) {
            map.grid(i, j) = terrainImage.at<double>(j, i);
            // std::cout <<terrainImage.at<double>(i, j) << " ";
        }
    }
    return map;
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
    // TODO: make these distributions member variables
    mParticles.resize(numParticles);
    for (size_t i = 0; i < numParticles; i++) {
        mParticles[i] = manif::SE2d(mXDistUniform(mRNG), mYDistUniform(mRNG), mThetaDistUniform(mRNG));
        // mParticles[i] = manif::SE2d::Random();
    }
}

// TODO: make it const
std::optional<Eigen::Vector3d> TerrainParticleFilter::get_surface_normal(manif::SE2d const& pose) {
    Eigen::Vector2i minCorner = position_to_idx(pose.translation() - mFootprint / 2.0);
    Eigen::Vector2i maxCorner = position_to_idx(pose.translation() + mFootprint / 2.0);
    Eigen::Vector2i footprintCells = maxCorner - minCorner;
    // TODO: index with seq if eigen 3.4 is available
    Eigen::MatrixXd neighborhood = mTerrainMap.grid.block(minCorner.x(), minCorner.y(),
                                                          footprintCells.x(), footprintCells.y());
    if (neighborhood.rows() == 0 || neighborhood.cols() == 0) {
        // std::stringstream ss;
        // ss << "pose out of bounds of terrain map: " << pose.translation().transpose();
        // throw std::runtime_error(ss.str());
        return std::nullopt;
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
    return std::make_optional(normal);
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
        std::optional<Eigen::Vector3d> maybe_normal = get_surface_normal(mParticles[i]);
        if (!maybe_normal) {
            weights[i] = 0;
            continue;
        }
        Eigen::Vector3d normal = *maybe_normal;
        Eigen::Vector2d forward2d = mParticles[i].rotation() * Eigen::Vector2d::UnitX();
        Eigen::Vector3d particle_forward(forward2d.x(), forward2d.y(), 0);
        Eigen::Vector3d left = normal.cross(particle_forward);
        Eigen::Vector3d forward = left.cross(normal);
        Eigen::Matrix3d rotation;
        rotation.col(0) = forward.normalized();
        rotation.col(1) = left.normalized();
        // TODO: is this necessary?
        rotation.col(2) = normal.normalized();
        Eigen::Quaterniond particle_orientation(rotation);
        manif::SO3d particle_SO3(particle_orientation.normalized());
        manif::SO3d true_SO3(orientation);
        manif::SO3Tangentd difference_tangent = particle_SO3.lminus(true_SO3);
        double weight = std::exp(-0.5 * difference_tangent.coeffs().squaredNorm());
        // std::cout << "Rotation: " << std::endl << rotation << std::endl;
        // std::cout << "forward: " << forward2d.transpose() << std::endl;
        // Eigen::Vector3d zAxis = orientation * Eigen::Vector3d::UnitZ();
        // double weight = std::clamp(normal.dot(zAxis), 0.0, 1.0);
        weights[i] = weight;
    }

    // normalize weights and use them to update pose estimate
    weights.normalize();
    std::vector<double> weights_vec(weights.data(), weights.data() + weights.size());
    update_pose_estimate(mParticles, weights_vec);

    // resample particles
    std::discrete_distribution<size_t> distribution(weights_vec.begin(), weights_vec.end());
    std::vector<manif::SE2d> newParticles(mParticles.size());
    for (size_t i = 0; i < mParticles.size(); i++) {
        newParticles[i] = mParticles[distribution(mRNG)];
    }

    // add random noise to particles
    for (auto& particle : newParticles) {
        particle = particle.rplus(manif::SE2Tangentd(mResamplingXDist(mRNG), mResamplingXDist(mRNG), mResamplingThetaDist(mRNG)));
    }

    // replace a percentage of the particles with random particles
    auto numRandomParticles = static_cast<size_t>(mParticles.size() * mRandomInjectionRate);
    Eigen::Vector2d minCorner = idx_to_position(Eigen::Vector2i(0, 0));
    Eigen::Vector2d maxCorner = idx_to_position(Eigen::Vector2i(mTerrainMap.grid.cols() - 1, mTerrainMap.grid.rows() - 1));
    std::uniform_real_distribution<> xDist(minCorner.x(), maxCorner.x());
    std::uniform_real_distribution<> yDist(minCorner.y(), maxCorner.y());
    std::uniform_real_distribution<double> thetaDist(-M_PI, M_PI);
    for (size_t i = 0; i < numRandomParticles; i++) {
        newParticles[i] = manif::SE2d(xDist(mRNG), yDist(mRNG), thetaDist(mRNG));
    }
    mParticles = newParticles;
}

const manif::SE2d& TerrainParticleFilter::get_pose_estimate() const {
    return mPoseEstimate;
}

const std::vector<manif::SE2d>& TerrainParticleFilter::get_particles() const {
    return mParticles;
}

// TODO: make it const
const Eigen::MatrixXd& TerrainParticleFilter::get_terrain_grid() {
    return mTerrainMap.grid;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr TerrainParticleFilter::get_neighborhood() {
    return mNeighborhoodCloud;
}
