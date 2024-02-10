#include "lander_align.hpp"
#include "pch.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <cmath>
#include <cstddef>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

#include <lie/se3.hpp>
#include <math.h>
#include <optional>
#include <random>
#include <ros/subscriber.h>
#include <tuple>
#include <vector>

#include <lie/se3.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>


namespace mrover {

    auto LanderAlignNodelet::onInit() -> void {
        mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle();
        mThreshold = 10;
        mVectorSub = mNh.subscribe("/camera/left/points", 1, &LanderAlignNodelet::LanderCallback, this);
        mDebugVectorPub = mNh.advertise<geometry_msgs::Vector3>("/lander_align/Pose", 1);
        mTwistPub = mNh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);


        //TF Create the Reference Frames
        mNh.param<std::string>("camera_frame", mCameraFrameId, "zed2i_left_camera_frame");
        mNh.param<std::string>("world_frame", mMapFrameId, "map");
    }

    void LanderAlignNodelet::LanderCallback(sensor_msgs::PointCloud2Ptr const& cloud) {
        filterNormals(cloud);
        if (std::optional<Eigen::Vector3f> planeNorm = ransac(mFilteredPoints, 0.2, 10, 100); planeNorm) {
            geometry_msgs::Vector3 vect;
            vect.x = planeNorm.value().x();
            vect.y = planeNorm.value().y();
            vect.z = planeNorm.value().z();
            mDebugVectorPub.publish(vect);
        }
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
        mFilteredPoints.clear();
        auto* cloudData = reinterpret_cast<Point const*>(cloud->data.data());

        // define randomizer
        std::default_random_engine generator;
        std::uniform_int_distribution<int> distribution(0, (int) 200);

        for (auto point = cloudData; point < cloudData + (cloud->height * cloud->width); point += distribution(generator)) {
            bool isPointInvalid = (!std::isfinite(point->x) || !std::isfinite(point->y) || !std::isfinite(point->z));
            if (point->normal_z < mThreshold && !isPointInvalid) {
                mFilteredPoints.push_back(point);
            }
        }
    }

    auto LanderAlignNodelet::ransac(std::vector<Point const*> const& points, float const distanceThreshold, int minInliers, int const epochs) -> std::optional<Eigen::Vector3f> {
        // TODO: use RANSAC to find the lander face, should be the closest, we may need to modify this to output more information, currently the output is the normal
        // takes 3 samples for every epoch and terminates after specified number of epochs
        Eigen::Vector3f bestPlane(0, 0, 0); // normal vector representing plane (initialize as zero vector?? default ctor??)
        // Eigen::Vector3f currentCenter(0, 0, 0); // Keeps track of current center of plane in current epoch
        float offset;

        // define randomizer
        std::default_random_engine generator;
        std::uniform_int_distribution<int> distribution(0, (int) points.size() - 1);

        if (points.size() < 3) {
            return std::nullopt;
        }
        while (bestPlane.isZero()) { // TODO add give up condition after X iter
            for (int i = 0; i < epochs; ++i) {
                // currentCenter *= 0; // Set all vals in current center to zero at the start of each epoch
                // sample 3 random points (potential inliers)
                Point const* point1 = points[distribution(generator)];
                Point const* point2 = points[distribution(generator)];
                Point const* point3 = points[distribution(generator)];

                Eigen::Vector3f vec1{point1->x, point1->y, point1->z};
                Eigen::Vector3f vec2{point2->x, point2->y, point2->z};
                Eigen::Vector3f vec3{point3->x, point3->y, point3->z};

                // fit a plane to these points
                Eigen::Vector3f normal = (vec1 - vec2).cross(vec1 - vec3).normalized();
                offset = -normal.dot(vec1); // calculate offset (D value) using one of the points

                int numInliers = 0;

                for (auto p: points) {
                    // calculate distance of each point from potential plane
                    float distance = std::abs(normal.x() * p->x + normal.y() * p->y + normal.z() * p->z + offset);

                    if (distance < distanceThreshold) {
                        // Add points to current planes center
                        // currentCenter(0) += p->x;
                        // currentCenter(1) += p->y;
                        // currentCenter(2) += p->z;
                        ++numInliers; // count num of inliers that pass the "good enough fit" threshold
                    }
                }

                // update best plane if better inlier count
                if (numInliers > minInliers) {
                    minInliers = numInliers;
                    bestPlane = normal.normalized();

                    // If this is the new best plane, set mBestCenter
                    // mBestCenter = currentCenter / static_cast<float>(minInliers);
                }
            }
        }

        // Run through one more loop to identify the center of the plane (one possibility for determining best center)
        int numInliers = 0;
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

        if (numInliers == 0) {
            return std::nullopt;
        }

        mBestCenter /= static_cast<float>(numInliers);

        SE3 planeLoc{
                {mBestCenter(0), mBestCenter(1), mBestCenter(2)},
                {}};

        std::string immediateFrameId = "immediatePlane";
        SE3::pushToTfTree(mTfBroadcaster, immediateFrameId, mCameraFrameId, planeLoc);

        ROS_INFO("THE LOCATION OF THE PLANE IS AT: %f, %f, %f with normal vector %f, %f, %f", mBestCenter(0), mBestCenter(1), mBestCenter(2), bestPlane.x(), bestPlane.y(), bestPlane.z());

        return bestPlane;
    }

    auto LanderAlignNodelet::PID::rotate_speed(float theta) const -> float {
        return Angle_P * theta;
    }


    auto LanderAlignNodelet::PID::find_angle(Eigen::Vector3f current, Eigen::Vector3f target) -> float {
        Eigen::Vector3f u1 = current.normalized();
        Eigen::Vector3f u2 = target.normalized();
        float theta = acos(u1.dot(u2));
        float perp_alignment = u2[0] * -u1[1] + u2[1] * u1[0];
        if (perp_alignment > 0) {
            return theta;
        }
        return -theta;
    }

    auto LanderAlignNodelet::PID::drive_speed(float distance) -> float {
        return distance * Linear_P;
    }

    auto LanderAlignNodelet::PID::find_distance(Eigen::Vector3f current, Eigen::Vector3f target) -> float {
        Eigen::Vector3f difference = target - current;
        float distance = difference.norm();
        return distance;
    }

    // auto LanderAlignNodelet::PID::calculate(Eigen::Vector3f& input, Eigen::Vector3f& target) -> std::tuple<float> {
    //     input[2] = 0;
    //     target[2] = 0;

    //     return {rotate_command(input, target), drive_speed(input, target)};
    // }


    void LanderAlignNodelet::sendTwist(Eigen::Vector3f const& planeNormal, Eigen::Vector3f const& planeCenterInWorld, Eigen::Vector3f const& offset) {
        SE3 rover;

        geometry_msgs::Twist twist;

        float const linear_thresh = 0.1; // could be member variables
        float const angular_thresh = 0.1;

        Eigen::Vector3f rover_dir;

        Eigen::Vector3f targetPosInWorld = planeCenterInWorld - offset;
        targetPosInWorld[2] = 0;

        PID pid(0.1, 0.1); // literally just P -- ugly class and probably needs restructuring in the future

        ros::Rate rate(20); // ROS Rate at 20Hz
        while (ros::ok()) {
            rover = SE3::fromTfTree(mTfBuffer, "map", "base_link");
            Eigen::Vector3f roverPosInWorld = rover.position();
            roverPosInWorld[2] = 0;

            switch (mLoopState) {
                case RTRSTATE::turn1: {
                    rover_dir = rover.rotation().matrix().col(0);
                    rover_dir[2] = 0;
                    float angle = pid.find_angle(rover_dir, targetPosInWorld - roverPosInWorld);
                    float angle_rate = pid.rotate_speed(angle);

                    twist.angular.z = angle_rate;

                    if (abs(angle) < angular_thresh) {
                        mLoopState = RTRSTATE::drive;
                    }
                }

                case RTRSTATE::drive: {
                    Eigen::Vector3f roverPosInWorld = rover.position();
                    float distance = pid.find_distance(roverPosInWorld, targetPosInWorld - roverPosInWorld);
                    float drive_rate = pid.drive_speed(distance);

                    twist.linear.x = drive_rate;

                    if (abs(distance) < linear_thresh) {
                        mLoopState = RTRSTATE::turn2;
                    }
                }

                case RTRSTATE::turn2: {
                    rover_dir = rover.rotation().matrix().col(0);
                    rover_dir[2] = 0;
                    float angle = pid.find_angle(rover_dir, planeNormal);
                    float angle_rate = pid.rotate_speed(angle);

                    twist.angular.z = angle_rate;

                    if (abs(angle) < angular_thresh) {
                        mLoopState = RTRSTATE::done;
                    }
                }

                case RTRSTATE::done:
                    break;
            }
            mTwistPub.publish(twist);

            ROS_INFO("Running turn/drive state machine...");
            rate.sleep();
        }
    }
} // namespace mrover