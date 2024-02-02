#include "lander_align.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <random>
#include <vector>

namespace mrover {

    auto LanderAlignNodelet::onInit() -> void {
        mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle();
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

    auto LanderAlignNodelet::filterNormals(sensor_msgs::PointCloud2Ptr const& cloud, int const threshold) -> std::vector<Point*> {
        // TODO: OPTIMIZE; doing this many push_back calls could slow things down

        auto* cloudData = reinterpret_cast<Point const*>(cloud->fields.data());
        std::vector<Point*> extractedPoints;

        for (auto point = cloudData; point < cloudData + (cloud->height * cloud->width); point++) {
            if (point->normal_z < threshold) {
                extractedPoints.push_back(point);
            }
        }


        return extractedPoints;
    }

    auto LanderAlignNodelet::ransac(std::vector<Point*> const& points, double const distanceThreshold, int const epochs) -> Eigen::Vector3f {
        // TODO: use RANSAC to find the lander face, should be the closest, we may need to modify this to output more information, currently the output is the normal
        // takes 3 samples for every epoch and terminates after specified number of epochs
        Eigen::Vector3f bestPlane; // normal vector representing plane

        // define randomizer
        std::default_random_engine generator;
        std::uniform_int_distribution<int> distribution(0, (int) points.size() - 1);

        for (int i = 0; i < epochs; ++i) {
            // sample 3 random points (potential inliers)
            Point* point1 = points[distribution(generator)];
            Point* point2 = points[distribution(generator)];
            Point* point3 = points[distribution(generator)];

            Eigen::Vector3f vec1{point1->x, point1->y, point1->z};
            Eigen::Vector3f vec2{point2->x, point2->y, point2->z};
            Eigen::Vector3f vec3{point3->x, point3->y, point3->z};

            // fit a plane to these points TODO change this to vec substract then cross
            Eigen::Vector3f u{point1->x - point2->x, point1->y - point2->y, point1->z - point2->z};
            Eigen::Vector3f v{point1->x - point3->x, point1->y - point3->y, point1->z - point3->z};
            // Eigen::Vector3f plane = u.subTo(Dest &dst)
        }
    }

} // namespace mrover
