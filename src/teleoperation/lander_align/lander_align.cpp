#include "lander_align.hpp"
#include "pch.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <geometry_msgs/Vector3.h>
#include <random>
#include <ros/subscriber.h>
#include <vector>

namespace mrover {

    auto LanderAlignNodelet::onInit() -> void {
        mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle();
        mThreshold = 10;
        mVectorSub = mNh.subscribe("/camera/left/points", 1, &LanderAlignNodelet::filterNormals, this);
        mDebugVectorPub = mNh.advertise<geometry_msgs::Vector3>("/lander_align/Pose", 1);
        mBestCenter(0, 0, 0);
    }

    void LanderAlignNodelet::LanderCallback(sensor_msgs::PointCloud2Ptr const& cloud) {
        filterNormals(cloud);
        Eigen::Vector3f planeNorm = ransac(mFilteredPoints, 1, 10, 10);

        geometry_msgs::Vector3 vect;
        vect.x = planeNorm.x();
        vect.y = planeNorm.y();
        vect.z = planeNorm.z();

        mDebugVectorPub.publish(vect);
    }

    // deprecated/not needed anymore
    // auto LanderAlignNodelet::downsample(sensor_msgs::PointCloud2Ptr const& cloud) -> sensor_msgs::PointCloud2Ptr {
    // TODO: make a new point cloud with half the resolution, check the zed wrapper for how to make one
    // sensor_msgs::PointCloud2 cloudSample;
    // cloudSample.header = cloud->header
    //     cloudSample.height = cloud->height / 2;
    //     cloudSample.width = cloud->width / 2;
    //     cloudSample.fields
    // }

    void LanderAlignNodelet::filterNormals(sensor_msgs::PointCloud2Ptr const& cloud) {
        // TODO: OPTIMIZE; doing this maobject_detector/debug_imgny push_back calls could slow things down

        auto* cloudData = reinterpret_cast<Point const*>(cloud->fields.data());
        // std::vector<Point const*> extractedPoints;

        for (auto point = cloudData; point < cloudData + (cloud->height * cloud->width); point++) {
            if (point->normal_z < mThreshold) {
                mFilteredPoints.push_back(point);
            }
        }
    }

    auto LanderAlignNodelet::ransac(std::vector<Point const*> const& points, float const distanceThreshold, int minInliers, int const epochs) -> Eigen::Vector3f {
        // TODO: use RANSAC to find the lander face, should be the closest, we may need to modify this to output more information, currently the output is the normal
        // takes 3 samples for every epoch and terminates after specified number of epochs
        Eigen::Vector3f bestPlane(0, 0, 0); // normal vector representing plane (initialize as zero vector?? default ctor??)

        // define randomizer
        std::default_random_engine generator;
        std::uniform_int_distribution<int> distribution(0, (int) points.size() - 1);

        do {
            for (int i = 0; i < epochs; ++i) {
                // sample 3 random points (potential inliers)
                Point const* point1 = points[distribution(generator)];
                Point const* point2 = points[distribution(generator)];
                Point const* point3 = points[distribution(generator)];

                Eigen::Vector3f vec1{point1->x, point1->y, point1->z};
                Eigen::Vector3f vec2{point2->x, point2->y, point2->z};
                Eigen::Vector3f vec3{point3->x, point3->y, point3->z};

                // fit a plane to these points
                Eigen::Vector3f normal = (vec1 - vec2).cross(vec1 - vec3).normalized();
                float offset = -normal.dot(vec1); // calculate offset (D value) using one of the points

                int numInliers = 0;

                for (auto p: points) {
                    // calculate distance of each point from potential plane
                    float distance = std::abs(normal.x() * p->x + normal.y() * p->y + normal.z() * p->z + offset);

                    if (distance < distanceThreshold) {
                        ++numInliers; // count num of inliers that pass the "good enough fit" threshold
                    }
                }

                // update best plane if better inlier count
                if (numInliers > minInliers) {
                    minInliers = numInliers;
                    bestPlane = normal;
                    mBestOffset = offset;
                }
            }
        } while (bestPlane.isZero()); // keep trying until we get valid result TODO break case after X attempts??

        // Run through one more loop to identify the center of the plane
        for (auto p: points) {
            // calculate distance of each point from potential plane
            float distance = std::abs(bestPlane.x() * p->x + bestPlane.y() * p->y + bestPlane.z() * p->z + mBestOffset);

            if (distance < distanceThreshold) {
                mBestCenter(0) += p->x;
                mBestCenter(1) += p->y;
                mBestCenter(2) += p->z;
                ++numInliers; // count num of inliers that pass the "good enough fit" threshold
            }
        }

        mBestCenter /= numInliers;

        return bestPlane;
    }


} // namespace mrover