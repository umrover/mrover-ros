#include "lander_align.hpp"
#include "lie.hpp"
#include "pch.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

#include <manif/impl/se3/SE3.h>
#include <memory>
#include <opencv4/opencv2/core/hal/interface.h>
#include <optional>
#include <point.hpp>
#include <random>
#include <ros/duration.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>
#include <tuple>
#include <unistd.h>
#include <vector>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/QR>

namespace mrover {

    auto LanderAlignNodelet::onInit() -> void {
        mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle();
        mZThreshold = .1;
        mVectorSub = mNh.subscribe("/camera/left/points", 1, &LanderAlignNodelet::LanderCallback, this);
        mDebugVectorPub = mNh.advertise<geometry_msgs::Vector3>("/lander_align/Pose", 1);
        mTwistPub = mNh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        mDebugPCPub = mNh.advertise<sensor_msgs::PointCloud2>("/lander_align/debugPC", 1);


        //TF Create the Reference Frames
        mNh.param<std::string>("camera_frame", mCameraFrameId, "zed_left_camera_frame");
        mNh.param<std::string>("world_frame", mMapFrameId, "map");

        mBestLocationInWorld = std::make_optional<Eigen::Vector3d>(0, 0, 0);
        mBestNormalInWorld = std::make_optional<Eigen::Vector3d>(0, 0, 0);
    }

    void LanderAlignNodelet::LanderCallback(sensor_msgs::PointCloud2Ptr const& cloud) {
        filterNormals(cloud);
        ransac(0.2, 10, 100);
        if (mBestNormalInZED.has_value()) {
            geometry_msgs::Vector3 vect;
            vect.x = mBestNormalInZED.value().x();
            vect.y = mBestNormalInZED.value().y();
            vect.z = mBestNormalInZED.value().z();
            mDebugVectorPub.publish(vect);
            //sendTwist(10.0);
        }
    }

    void LanderAlignNodelet::filterNormals(sensor_msgs::PointCloud2Ptr const& cloud) {
        // TODO: OPTIMIZE; doing this maobject_detector/debug_imgny push_back calls could slow things down
        mFilteredPoints.clear();
        auto* cloudData = reinterpret_cast<Point const*>(cloud->data.data());
        ROS_INFO("Point cloud size: %i", cloud->height * cloud->width);

        // define randomizer
        std::default_random_engine generator;
        std::uniform_int_distribution<int> pointDistribution(0, (int) mLeastSamplingDistribution);

        for (auto point = cloudData; point < cloudData + (cloud->height * cloud->width); point += pointDistribution(generator)) {
            bool isPointInvalid = (!std::isfinite(point->x) || !std::isfinite(point->y) || !std::isfinite(point->z));
            if (abs(point->normal_z) < mZThreshold && !isPointInvalid) {
                mFilteredPoints.push_back(point);
                // ROS_INFO("Filtered point: %f, %f, %f", point->normal_x, point->normal_y, point->normal_z);
            }
        }
    }

	void LanderAlignNodelet::uploadPC(int numInliers, double distanceThreshold){
        auto debugPointCloudPtr = boost::make_shared<sensor_msgs::PointCloud2>();
        fillPointCloudMessageHeader(debugPointCloudPtr);
        debugPointCloudPtr->is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
        debugPointCloudPtr->is_dense = true;
        debugPointCloudPtr->height = 1;
        debugPointCloudPtr->width = numInliers;
        debugPointCloudPtr->header.seq = 0;
        debugPointCloudPtr->header.stamp = ros::Time();
        debugPointCloudPtr->header.frame_id = "zed_left_camera_frame";
        debugPointCloudPtr->data.resize((numInliers * sizeof(Point))/sizeof(uchar));
        auto pcPtr = reinterpret_cast<Point*>(debugPointCloudPtr->data.data());
        size_t i = 0;
        for (auto p: mFilteredPoints) {
            // calculate distance of each point from potential plane
            double distance = std::abs(mBestNormalInZED.value().x() * p->x + mBestNormalInZED.value().y() * p->y + mBestNormalInZED.value().z() * p->z + mBestOffset);
            if (distance < distanceThreshold) {
                pcPtr[i].x = p->x;
                pcPtr[i].y = p->y;
                pcPtr[i].z = p->z;
                pcPtr[i].b = p->b;
                pcPtr[i].g = p->g;
                pcPtr[i].r = p->r;
                pcPtr[i].a = p->a;
                ++i;
            }
        }
        mDebugPCPub.publish(debugPointCloudPtr);
	}

    void LanderAlignNodelet::ransac(double const distanceThreshold, int minInliers, int const epochs) {
        // TODO: use RANSAC to find the lander face, should be the closest, we may need to modify this to output more information, currently the output is the normal
        // takes 3 samples for every epoch and terminates after specified number of epochs
        // Eigen::Vector3d mBestNormal(0, 0, 0); // normal vector representing plane (initialize as zero vector?? default ctor??)
        // Eigen::Vector3d currentCenter(0, 0, 0); // Keeps track of current center of plane in current epoch
        double offset;

        // define randomizer
        std::default_random_engine generator;
        std::uniform_int_distribution<int> distribution(0, (int) mFilteredPoints.size() - 1);

        if (mFilteredPoints.size() < 3) {
            mBestNormalInZED = std::nullopt;
            mBestLocationInZED = std::nullopt;
            return;
        }


        mBestNormalInZED = std::make_optional<Eigen::Vector3d>(0, 0, 0);
        mBestLocationInZED = std::make_optional<Eigen::Vector3d>(0, 0, 0);

        int numInliers = 0;
        while (mBestNormalInZED.value().isZero()) { // TODO add give up condition after X iter
            ROS_INFO("in zero loop");
            for (int i = 0; i < epochs; ++i) {
                // currentCenter *= 0; // Set all vals in current center to zero at the start of each epoch
                // sample 3 random points (potential inliers)
                Point const* point1 = mFilteredPoints[distribution(generator)];
                Point const* point2 = mFilteredPoints[distribution(generator)];
                Point const* point3 = mFilteredPoints[distribution(generator)];

                Eigen::Vector3d vec1{point1->x, point1->y, point1->z};
                Eigen::Vector3d vec2{point2->x, point2->y, point2->z};
                Eigen::Vector3d vec3{point3->x, point3->y, point3->z};

                // fit a plane to these points
                Eigen::Vector3d normal = (vec1 - vec2).cross(vec1 - vec3).normalized();
                offset = -normal.dot(vec1); // calculate offset (D value) using one of the points

                numInliers = 0;

                // assert(normal.x() != 0 && normal.y() != 0 && normal.z() != 0);
                // In some situations we get the 0 vector with surprising frequency

                for (auto p: mFilteredPoints) {
                    // calculate distance of each point from potential plane
                    double distance = std::abs(normal.x() * p->x + normal.y() * p->y + normal.z() * p->z + offset); // 

                    if (distance < distanceThreshold) {
                        // Add points to current planes center
                        // currentCenter(0) += p->x;
                        // currentCenter(1) += p->y;
                        // currentCenter(2) += p->z;
                        ++numInliers; // count num of inliers that pass the "good enough fit" threshold
                    }
                }

                // update best plane if better inlier count
                if (numInliers > minInliers && normal.x() != 0 && normal.y() != 0 && normal.z() != 0) {
                    minInliers = numInliers;
                    mBestNormalInZED.value() = normal;
                    mBestOffset = offset;

                    // If this is the new best plane, set mBestCenterInZED
                    // mBestCenterInZED = currentCenter / static_cast<float>(minInliers);
                }
            }
        }

        // Run through one more loop to identify the center of the plane (one possibility for determining best center)
        numInliers = 0;
        for (auto p: mFilteredPoints) {
            // calculate distance of each point from potential plane
            double distance = std::abs(mBestNormalInZED.value().x() * p->x + mBestNormalInZED.value().y() * p->y + mBestNormalInZED.value().z() * p->z + mBestOffset);
            if (distance < distanceThreshold) {
                mBestLocationInZED.value().x() += p->x;
                mBestLocationInZED.value().y() += p->y;
                mBestLocationInZED.value().z() += p->z;
                ++numInliers; // count num of inliers that pass the "good enough fit" threshold
            }
        }

        if (numInliers == 0) {
            ROS_INFO("zero inliers");

            mBestNormalInZED = std::nullopt;
            mBestLocationInZED = std::nullopt;
            return;
        }

        //Average pnts
        mBestLocationInZED.value() /= static_cast<float>(numInliers);

        if(mBestNormalInZED.value().x() > 0) mBestNormalInZED.value() *=-1;

        mBestOffsetInZED =  std::make_optional<Eigen::Vector3d>(mBestLocationInZED.value() + mBestNormalInZED.value());

		uploadPC(numInliers, distanceThreshold);

        manif::SE3d zedToMap = SE3Conversions::fromTfTree(mTfBuffer, mCameraFrameId, mMapFrameId);
		
		//Calculate the normal in the world frame
        mBestNormalInWorld = std::make_optional<Eigen::Vector3d>(zedToMap.rotation().matrix() * mBestNormalInZED.value());

        //Calculate the SO3 in the world frame
        Eigen::Matrix3d rot;
        {
            Eigen::Vector3d n = mBestNormalInWorld.value().normalized();
			ROS_INFO("normal x: %.7f", static_cast<double>(mBestNormalInZED.value().x()));

            // y = dummy.cross(mBestNormalInZED.value()).normalized();
            // z = x.cross(mBestNormalInZED.value()).normalized();
            rot <<  n.x(),0,0,
                    n.y(),0,1,
                    n.z(),1,0;
                                std::cout << "rot matrix 2 s" << std::endl << rot << std::endl;

            
            Eigen::HouseholderQR<Eigen::Matrix3d> qr{rot};
            Eigen::Matrix3d q = qr.householderQ();
            rot.col(0) = q.col(0);
            rot.col(1) = q.col(2);
            rot.col(2) = q.col(1);
            std::cout << "rot matrix " << std::endl << rot << std::endl;
        }
		//Calculate the plane location in the world frame
		manif::SE3d plane_loc_in_world = {{mBestLocationInZED.value().x(), mBestLocationInZED.value().y(), mBestLocationInZED.value().z()}, manif::SO3d{Eigen::Quaterniond{rot}.normalized()}};
		plane_loc_in_world = zedToMap * plane_loc_in_world;
        manif::SE3d plane_loc_in_world_final = {{plane_loc_in_world.translation().x(),plane_loc_in_world.translation().y(),plane_loc_in_world.translation().z()}, manif::SO3d{Eigen::Quaterniond{rot}.normalized()}};

        //Calculate the offset location in the world frame
		manif::SE3d offset_loc_in_world = {{mBestOffsetInZED.value().x(), mBestOffsetInZED.value().y(), mBestOffsetInZED.value().z()}, manif::SO3d{Eigen::Quaterniond{rot}.normalized()}};
		offset_loc_in_world = zedToMap * offset_loc_in_world;
        manif::SE3d offset_loc_in_world_final = {{offset_loc_in_world.translation().x(),offset_loc_in_world.translation().y(),offset_loc_in_world.translation().z()}, manif::SO3d{Eigen::Quaterniond{rot}.normalized()}};


		//Push to the tf tree
        SE3Conversions::pushToTfTree(mTfBroadcaster, "plane", mMapFrameId, plane_loc_in_world_final);
        SE3Conversions::pushToTfTree(mTfBroadcaster, "offset", mMapFrameId, offset_loc_in_world_final);

        //For the normal
        manif::SE3d mNormalLocInWorld = {{mBestNormalInWorld.value().x(), mBestNormalInWorld.value().y(), mBestNormalInWorld.value().z()}, manif::SO3d{Eigen::Quaterniond{rot}.normalized()}};//TODO: THIS IS A RANDOM ROTATION MATRIX

        SE3Conversions::pushToTfTree(mTfBroadcaster, "normalInWorld", mMapFrameId, mNormalLocInWorld);

        //For the normal
        manif::SE3d mNormalLocInZED = {{mBestNormalInZED.value().x(), mBestNormalInZED.value().y(), mBestNormalInZED.value().z()}, manif::SO3d{Eigen::Quaterniond{rot}.normalized()}};//TODO: THIS IS A RANDOM ROTATION MATRIX

        SE3Conversions::pushToTfTree(mTfBroadcaster, "normalInZED", mMapFrameId, mNormalLocInZED);


        ROS_INFO("Max inliers: %i", minInliers);
        ROS_INFO("THE LOCATION OF THE PLANE IS AT: %f, %f, %f with normal vector %f, %f, %f", mBestLocationInZED.value().x(), mBestLocationInZED.value().y(), mBestLocationInZED.value().z(), mBestNormalInWorld.value().x(), mBestNormalInWorld.value().y(), mBestNormalInWorld.value().z());
    }

    auto LanderAlignNodelet::PID::rotate_speed(float theta) const -> float {
        return Angle_P * theta;
    }


    auto LanderAlignNodelet::PID::find_angle(Eigen::Vector3d const& current, Eigen::Vector3d const& target) -> float {
        Eigen::Vector3d u1 = current.normalized();
        u1.z() = 0;
        Eigen::Vector3d u2 = target.normalized();
        u2.z() = 0;
        float theta = fmod(acos(u1.dot(u2)), static_cast<float>(180));
        float perp_alignment = u2[0] * -u1[1] + u2[1] * u1[0];
        if (perp_alignment > 0) {
            return theta;
        }
        return -theta;
    }

    auto LanderAlignNodelet::PID::drive_speed(float distance) -> float {
        return distance * Linear_P;
    }

    auto LanderAlignNodelet::PID::find_distance(Eigen::Vector3d const& current, Eigen::Vector3d const& target) -> double {
        Eigen::Vector3d difference = target - current;
        double distance = difference.norm();
        return distance;
    }

    LanderAlignNodelet::PID::PID(float angle_P, float linear_P) : Angle_P(angle_P), Linear_P(linear_P) {
    }

    // auto LanderAlignNodelet::PID::calculate(Eigen::Vector3d& input, Eigen::Vector3d& target) -> std::tuple<float> {
    //     input[2] = 0;
    //     target[2] = 0;

    //     return {rotate_command(input, target), drive_speed(input, target)};
    // }


    void LanderAlignNodelet::sendTwist(float const& offset) {

        //Locations
        manif::SE3d rover;
        Eigen::Vector3d rover_dir;
        Eigen::Vector3d targetPosInWorld = mBestLocationInWorld.value() + mBestNormalInWorld.value() * offset;

        //Final msg
        geometry_msgs::Twist twist;

        //Thr
        float const linear_thresh = 0.1; // could be member variables
        float const angular_thresh = 0.1;


        targetPosInWorld.z() = 0;
        PID pid(0.1, 0.1); // literally just P -- ugly class and probably needs restructuring in the future
        ros::Rate rate(20); // ROS Rate at 20Hz
        while (ros::ok()) {
            rover = SE3Conversions::fromTfTree(mTfBuffer, "base_link", "map");
            Eigen::Vector3d roverPosInWorld{static_cast<float>(rover.translation().x()), static_cast<float>(rover.translation().y()), 0};

            switch (mLoopState) {
                case RTRSTATE::turn1: {
                    rover_dir = {static_cast<float>(rover.rotation().matrix().col(0).x()), static_cast<float>(rover.rotation().matrix().col(0).y()), 0};
                    float angle = pid.find_angle(rover_dir, targetPosInWorld - roverPosInWorld);
                    float angle_rate = pid.rotate_speed(angle);

                    twist.angular.z = angle_rate;

                    if (abs(angle) < angular_thresh) {
                        mLoopState = RTRSTATE::drive;
                    }
                    // ROS_INFO("In state: turning to point...");
                }

                case RTRSTATE::drive: {
                    float distance = pid.find_distance(roverPosInWorld, targetPosInWorld - roverPosInWorld);
                    float drive_rate = pid.drive_speed(distance);

                    twist.linear.x = drive_rate;

                    if (abs(distance) < linear_thresh) {
                        mLoopState = RTRSTATE::turn2;
                    }
                    // ROS_INFO("In state: driving to point...");
                }

                case RTRSTATE::turn2: {
                    rover_dir = {static_cast<float>(rover.rotation().matrix().col(0).x()), static_cast<float>(rover.rotation().matrix().col(0).y()), 0};
                    rover_dir[2] = 0;
                    float angle = pid.find_angle(rover_dir, -mBestNormalInWorld.value()); // normal is pointing "towards" the rover, so we need to flip it to find angle
                    float angle_rate = pid.rotate_speed(angle);

                    twist.angular.z = angle_rate;

                    if (abs(angle) < angular_thresh) {
                        mLoopState = RTRSTATE::done;
                    }
                    // ROS_INFO("In state: turning to lander...");
                }

                case RTRSTATE::done:
                    break;
            }
            ROS_INFO("THE TWIST IS: Angular: %f, with linear %f,", twist.angular.z, twist.linear.x);

            mTwistPub.publish(twist);

            rate.sleep();
        }
    }
} // namespace mrover
