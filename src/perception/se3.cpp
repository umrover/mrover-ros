#include "se3.hpp"

#include <utility>

SE3::SE3(Vector position, SO3 rotation) : position(std::move(position)), rotation(std::move(rotation)) {
}

SE3 SE3::fromPosQuat(Vector const& position, Quaternion const& rotation) {
    return SE3{position, SO3{rotation}};
}

SE3 SE3::applyLeft(SE3 const& other) {
    auto affine = Eigen::Affine3d::Identity();
    affine.translate(other.position);
    affine.rotate(other.rotation.quaternion);
    return {affine * position, SO3{other.rotation.quaternion * rotation.quaternion}};
}

SE3 SE3::applyRight([[maybe_unused]] SE3 const& other) {
    throw std::logic_error("Not implemented yet!");
}

SE3 SE3::fromTfTree(tf2_ros::Buffer const& buffer, std::string const& parentFrame, std::string const& childFrame) {
    geometry_msgs::TransformStamped transform = buffer.lookupTransform(parentFrame, childFrame, ros::Time(0));
    return SE3::fromTf(transform.transform);
}

void SE3::publishToTfTree(tf2_ros::TransformBroadcaster& broadcaster, std::string const& childFrame, std::string const& parentFrame) {
    broadcaster.sendTransform(toTransformStamped(parentFrame, childFrame));
}

double SE3::posDistanceTo(SE3 const& other) {
    return (position - other.position).squaredNorm();
}

SE3::Vector const& SE3::posVector() const {
    return position;
}

bool SE3::isApprox(SE3 const& other, double tolerance) {
    return (position - other.position).norm() < tolerance && rotation.isApprox(other.rotation, tolerance);
}
