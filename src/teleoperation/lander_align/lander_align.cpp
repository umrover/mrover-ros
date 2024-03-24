#include "lander_align.hpp"
#include "mrover/LanderAlignActionFeedback.h"
#include "mrover/LanderAlignActionResult.h"
#include <algorithm>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <cmath>
#include <complex>
#include <optional>
#include <ros/rate.h>
#include <ros/topic.h>

namespace mrover {
    auto operator<<(std::ostream& ostream, RTRSTATE state) -> std::ostream& {
        return ostream << RTRSTRINGS[static_cast<std::size_t>(state)];
    }

    auto LanderAlignNodelet::onInit() -> void {
        mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle();
        mZThreshold = .5;
        mXThreshold = .1;
        mPlaneOffsetScalar = 2.5;
        mDebugVectorPub = mNh.advertise<geometry_msgs::Vector3>("/lander_align/Pose", 1);
        mTwistPub = mNh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        mDebugPCPub = mNh.advertise<sensor_msgs::PointCloud2>("/lander_align/debugPC", 1);

        mActionServer.emplace(mNh, "LanderAlignAction", [&](LanderAlignGoalConstPtr const& goal) { ActionServerCallBack(); }, false);
        mActionServer.value().start();
        
        //TF Create the Reference Frames
        mNh.param<std::string>("camera_frame", mCameraFrameId, "zed_left_camera_frame");
        mNh.param<std::string>("world_frame", mMapFrameId, "map");

        mPlaneLocationInWorldVector = std::make_optional<Eigen::Vector3d>(0, 0, 0);
        mNormalInWorldVector = std::make_optional<Eigen::Vector3d>(0, 0, 0);
        mLoopState = RTRSTATE::turn1;
    }

    auto LanderAlignNodelet::ActionServerCallBack() -> void {
        // mActionServer->acceptNewGoal();

        LanderAlignResult result;
        mPlaneOffsetScalar = 2.5;

        //If we haven't yet defined the point cloud we are working with
		mCloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/left/points", mNh);
		filterNormals(mCloud);
		ransac(0.1, 10, 100);
        
        //If there is a proper normal to drive to
        if (mNormalInZEDVector.has_value()) {
            sendTwist();
        }

        mPlaneOffsetScalar = 1;
        mLoopState = RTRSTATE::turn1;

        //If we haven't yet defined the point cloud we are working with
		mCloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/left/points", mNh);
		filterNormals(mCloud);
		ransac(0.1, 10, 100);
        
        //If there is a proper normal to drive to
        if (mNormalInZEDVector.has_value()) {
            sendTwist();
        }

        if(mActionServer->isPreemptRequested()){
            mActionServer->setPreempted();
        }else{
            mActionServer->setSucceeded(result);
        }
    }


    //Returns angle (yaw) around the z axis
    auto LanderAlignNodelet::calcAngleWithWorldX(Eigen::Vector3d xHeading) -> double { //I want to be editing this variable so it should not be const or &
        xHeading.z() = 0;
        xHeading.normalize();
        Eigen::Vector3d xAxisWorld{1, 0, 0};
        double angle = std::acos(xHeading.dot(xAxisWorld));
        if (xHeading.y() >= 0) {
            return angle;
        } else {
            return angle + std::numbers::pi;
        }
    }

    void LanderAlignNodelet::calcMotion(double desiredVelocity, double desiredOmega) {
        SE3d roverInWorld = SE3Conversions::fromTfTree(mTfBuffer, "base_link", "map");
        Eigen::Vector3d xAxisRotation = roverInWorld.rotation().col(0);
        double roverHeading = calcAngleWithWorldX(xAxisRotation);
        double targetHeading = calcAngleWithWorldX(-mNormalInWorldVector.value());

        //Current State
        Eigen::Vector3d currState{roverInWorld.translation().x(), roverInWorld.translation().y(), roverHeading};

        //Target State
        Eigen::Vector3d tarState{mOffsetLocationInWorldVector.value().x(), mOffsetLocationInWorldVector.value().y(), targetHeading};

        //Constants
        double Kx = 1;
        double Ky = 1;
        double Ktheta = 1;

        Eigen::Matrix3d rotation;
        rotation << std::cos(roverHeading), std::sin(roverHeading), 0,
                -std::sin(roverHeading), std::cos(roverHeading), 0,
                0, 0, 1;

        Eigen::Vector3d errState = rotation * (tarState - currState);

        //I think this is rad/s
        double zRotation = desiredOmega + (desiredVelocity / 2) * (Ky * (errState.y() + Ktheta * errState.z()) + (1 / Ky) * std::sin(errState.z()));

        //I think this is unit/s
        double xSpeed = Kx * errState.x() + desiredVelocity * std::cos(errState.z()) - Ktheta * errState.z() * zRotation;

        geometry_msgs::Twist twist;
        twist.angular.z = zRotation;
        twist.linear.x = xSpeed;

        mTwistPub.publish(twist);
    }

    void LanderAlignNodelet::filterNormals(sensor_msgs::PointCloud2ConstPtr const& cloud) {
        mFilteredPoints.clear();
        
        // Pointer to the underlying point cloud data
        auto* cloudData = reinterpret_cast<Point const*>(cloud->data.data());

        std::default_random_engine generator;
        std::uniform_int_distribution<int> pointDistribution(0, mLeastSamplingDistribution);

        // Loop over the entire PC
        for (auto point = cloudData; point < cloudData + (cloud->height * cloud->width); point += pointDistribution(generator)) {
            // Make sure all of the values are defined
            bool isPointInvalid = (!std::isfinite(point->x) || !std::isfinite(point->y) || !std::isfinite(point->z));
            if (!isPointInvalid && abs(point->normal_z) < mZThreshold && abs(point->normal_x) > mXThreshold) {
                mFilteredPoints.push_back(point);
            }
        }
    }

    void LanderAlignNodelet::uploadPC(int numInliers, double distanceThreshold) {
        auto debugPointCloudPtr = boost::make_shared<sensor_msgs::PointCloud2>();
        fillPointCloudMessageHeader(debugPointCloudPtr);
        debugPointCloudPtr->is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
        debugPointCloudPtr->is_dense = true;
        debugPointCloudPtr->height = 1;
        debugPointCloudPtr->width = numInliers;
        debugPointCloudPtr->header.seq = 0;
        debugPointCloudPtr->header.stamp = ros::Time();
        debugPointCloudPtr->header.frame_id = "zed_left_camera_frame";
        debugPointCloudPtr->data.resize(numInliers * sizeof(Point));
        auto pcPtr = reinterpret_cast<Point*>(debugPointCloudPtr->data.data());
        size_t i = 0;
        for (auto p: mFilteredPoints) {
            // calculate distance of each point from potential plane
            double distance = std::abs(mNormalInZEDVector.value().x() * p->x + mNormalInZEDVector.value().y() * p->y + mNormalInZEDVector.value().z() * p->z + mBestOffset);
            // double distance = std::abs(normal.x() * p->x + normal.y() * p->y + normal.z() * p->z + offset); //

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
        double offset;

        // define randomizer
        std::default_random_engine generator;
        std::uniform_int_distribution<int> distribution(0, (int) mFilteredPoints.size() - 1);

        if (mFilteredPoints.size() < 3) {
            mNormalInZEDVector = std::nullopt;
            mPlaneLocationInZEDVector = std::nullopt;
            return;
        }


        mNormalInZEDVector = std::make_optional<Eigen::Vector3d>(0, 0, 0);
        mPlaneLocationInZEDVector = std::make_optional<Eigen::Vector3d>(0, 0, 0);

        int numInliers = 0;
        while (mNormalInZEDVector.value().isZero()) { // TODO add give up condition after X iter
            for (int i = 0; i < epochs; ++i) {
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

                for (auto p: mFilteredPoints) {
                    // calculate distance of each point from potential plane
                    double distance = std::abs(normal.x() * p->x + normal.y() * p->y + normal.z() * p->z + offset); //

                    if (distance < distanceThreshold) {
                        ++numInliers; // count num of inliers that pass the "good enough fit" threshold
                    }
                }

                // update best plane if better inlier count
                if (numInliers > minInliers && normal.x() != 0 && normal.y() != 0 && normal.z() != 0) { // this don't make no sense
                    minInliers = numInliers;
                    mNormalInZEDVector.value() = normal;
                    mBestOffset = offset;
                }
            }
        }

        // Run through one more loop to identify the center of the plane (one possibility for determining best center)
        numInliers = 0;
        mPlaneLocationInZEDVector = std::make_optional<Eigen::Vector3d>(Eigen::Vector3d::Zero());
        for (auto p: mFilteredPoints) {
            // calculate distance of each point from potential plane
            double distance = std::abs(mNormalInZEDVector.value().x() * p->x + mNormalInZEDVector.value().y() * p->y + mNormalInZEDVector.value().z() * p->z + mBestOffset);
            if (distance < distanceThreshold) {
                mPlaneLocationInZEDVector.value().x() += p->x;
                mPlaneLocationInZEDVector.value().y() += p->y;
                mPlaneLocationInZEDVector.value().z() += p->z;
                ++numInliers; // count num of inliers that pass the "good enough fit" threshold
            }
        }

        if (numInliers == 0) {
            mNormalInZEDVector = std::nullopt;
            mPlaneLocationInZEDVector = std::nullopt;
            return;
        }

        //Average pnts
        mPlaneLocationInZEDVector.value() /= static_cast<float>(numInliers);


        uploadPC(numInliers, distanceThreshold);

        if (mNormalInZEDVector.value().x() > 0) mNormalInZEDVector.value() *= -1;


        mOffsetLocationInZEDVector = std::make_optional<Eigen::Vector3d>(mPlaneLocationInZEDVector.value() + mPlaneOffsetScalar * mNormalInZEDVector.value());

        SE3d zedToMap = SE3Conversions::fromTfTree(mTfBuffer, mCameraFrameId, mMapFrameId);

        //Calculate the SO3 in the world frame
        Eigen::Matrix3d rot;
        Eigen::Vector3d forward = mNormalInZEDVector.value().normalized();
        Eigen::Vector3d worldUp = Eigen::Vector3d::UnitZ();
        Eigen::Vector3d left = worldUp.cross(forward);
        Eigen::Vector3d up = forward.cross(left);
        ROS_INFO("THE LOCATION OF THE PLANE IS AT: %f, %f, %f with normal vector %f, %f, %f", mPlaneLocationInZEDVector.value().x(), mPlaneLocationInZEDVector.value().y(), mPlaneLocationInZEDVector.value().z(), forward.x(), forward.y(), forward.z());

        rot.col(0) = forward;
        rot.col(1) = left;
        rot.col(2) = up;

        //Calculate the plane location in the world frame
        SE3d mPlaneLocationInZEDSE3d = {{mPlaneLocationInZEDVector.value().x(), mPlaneLocationInZEDVector.value().y(), mPlaneLocationInZEDVector.value().z()}, SO3d{Eigen::Quaterniond{rot}.normalized()}};
        mPlaneLocationInWorldSE3d = zedToMap * mPlaneLocationInZEDSE3d;
        mPlaneLocationInWorldVector = std::make_optional<Eigen::Vector3d>(mPlaneLocationInWorldSE3d.translation());

        //Calculate the offset location in the world frame
        SE3d mOffsetLocationInZEDSE3d = {{mOffsetLocationInZEDVector.value().x(), mOffsetLocationInZEDVector.value().y(), mOffsetLocationInZEDVector.value().z()}, SO3d{Eigen::Quaterniond{rot}.normalized()}};
        mOffsetLocationInWorldSE3d = zedToMap * mOffsetLocationInZEDSE3d;
        mOffsetLocationInWorldVector = std::make_optional<Eigen::Vector3d>(mOffsetLocationInWorldSE3d.translation());

        //Push to the tf tree
        SE3Conversions::pushToTfTree(mTfBroadcaster, "plane", mMapFrameId, mPlaneLocationInWorldSE3d);
        SE3Conversions::pushToTfTree(mTfBroadcaster, "offset", mMapFrameId, mOffsetLocationInWorldSE3d);

        //Compare Rover Location to Target Location
        if(mOffsetLocationInZEDSE3d.translation().x() < 0) mNormalInZEDVector = std::nullopt;
    }

    auto LanderAlignNodelet::PID::rotate_speed(double theta) const -> double {
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


    void LanderAlignNodelet::sendTwist() {
        //Locations
        Eigen::Vector3d rover_dir;

        //Final msg
        geometry_msgs::Twist twist;

        //Threhsolds
        float const linear_thresh = 0.01; // could be member variables
        float const angular_thresh = 0.001;


        PID pid(0.1, 0.1);  // literally just P -- ugly class and probably needs restructuring in the future
        ros::Rate rate(20); // ROS Rate at 20Hzn::Matrix3d roverToPlaneNorm;
                            // Eigen::Vector3d Nup = Eigen::Vector3d::UnitZ();
                            // Eigen::Vector3d Nleft = Nup.cross(mBestNormalInWorld.value());

        // roverToPlaneNorm.col(0) = mBestNormalInWorld.value();
        // roverToPlaneNorm.col(1) = Nup;
        // roverToPlaneNorm.col(2) = Nleft;
        // manif::SO3d roverToP
        while (ros::ok()) {
            if(mActionServer->isPreemptRequested()){
                mActionServer->setPreempted();
                twist.angular.z = 0;
                twist.linear.x = 0;
                mTwistPub.publish(twist);
                break;
            }
                
        	LanderAlignFeedback feedback;
			feedback.curr_state = RTRSTRINGS[static_cast<std::size_t>(mLoopState)];
			mActionServer->publishFeedback(feedback);
            SE3Conversions::pushToTfTree(mTfBroadcaster, "plane", mMapFrameId, mPlaneLocationInWorldSE3d);
            SE3Conversions::pushToTfTree(mTfBroadcaster, "offset", mMapFrameId, mOffsetLocationInWorldSE3d);
            SE3d rover = SE3Conversions::fromTfTree(mTfBuffer, "base_link", "map");
            Eigen::Vector3d roverPosInWorld{(rover.translation().x()), (rover.translation().y()), 0.0};
            Eigen::Vector3d targetPosInWorld;
            if (mLoopState == RTRSTATE::turn1) {
                targetPosInWorld = mOffsetLocationInWorldVector.value();
            } else if (mLoopState == RTRSTATE::turn2) {
                targetPosInWorld = mPlaneLocationInWorldVector.value();
            }
            targetPosInWorld.z() = 0;

            Eigen::Vector3d roverToTargetForward = targetPosInWorld - roverPosInWorld;
            roverToTargetForward.normalize();

            Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
            Eigen::Vector3d left = up.cross(roverToTargetForward);

            Eigen::Matrix3d roverToTargetMat;
            roverToTargetMat.col(0) = roverToTargetForward;
            roverToTargetMat.col(1) = up;
            roverToTargetMat.col(2) = left;

            //SO3 Matrices for lie algebra
            SO3d roverToTargetSO3 = SE3Conversions::fromColumns(roverToTargetForward, left, up);
            SO3d roverSO3 = rover.asSO3();

            Eigen::Vector3d distanceToTargetVector = targetPosInWorld - Eigen::Vector3d{rover.translation().x(), rover.translation().y(), rover.translation().z()};

            double distanceToTarget = std::abs(distanceToTargetVector.norm());

            manif::SO3Tangentd SO3tan = roverToTargetSO3 - roverSO3; // 3 x 1 matrix of angular velocities (x,y,z)

            switch (mLoopState) {
                case RTRSTATE::turn1: {
                    if (distanceToTarget < 0.5) mLoopState = RTRSTATE::turn2;
                    

                    double angle_rate = mAngleP * SO3tan.z();
                    angle_rate = (std::abs(angle_rate) > mAngleFloor) ? angle_rate : copysign(mAngleFloor, angle_rate);

                    ROS_INFO("w_z velocity %f", SO3tan.z());

                    twist.angular.z = angle_rate;

                    if (std::abs(SO3tan.z()) < angular_thresh) {
                        mLoopState = RTRSTATE::drive;
                        twist.angular.z = 0;
                        twist.linear.x = 0;
                        ROS_INFO("Done spinning");
                    }
                    // ROS_INFO("In state: turning to point...");
                    break;
                }

                case RTRSTATE::drive: {
                    if (std::abs(SO3tan.z()) > angular_thresh) {
                        mLoopState = RTRSTATE::turn1;
                        ROS_INFO("Rotation got off");
                        twist.linear.x = 0;
                    }
                    double driveRate = std::min(mLinearP * distanceToTarget, 1.0);

                    ROS_INFO("distance: %f", distanceToTarget);

                    twist.linear.x = driveRate;

                    ROS_INFO("distance to target %f", distanceToTarget);

                    if (std::abs(distanceToTarget) < linear_thresh) {
                        mLoopState = RTRSTATE::turn2;
                        twist.linear.x = 0;
                        twist.angular.z = 0;
                    }
                    // ROS_INFO("In state: driving to point...");
                    break;
                }

                case RTRSTATE::turn2: {
                    double angle_rate = mAngleP * SO3tan.z();
                    angle_rate = (std::abs(angle_rate) > mAngleFloor) ? angle_rate : copysign(mAngleFloor, angle_rate);
                    twist.angular.z = angle_rate;
                    twist.linear.x = 0;


                    if (std::abs(SO3tan.z()) < angular_thresh) {
                        mLoopState = RTRSTATE::done;
                        twist.angular.z = 0;
                        twist.linear.x = 0;
                        //  ROS_INFO("Done turning to lander");
                    }
                    // ROS_INFO("In state: turning to lander...");
                    break;
                }

                case RTRSTATE::done: {
                    twist.linear.x = 0;
                    twist.angular.z = 0;
                    break;
					mCloud = nullptr;
                }
            }
            // ROS_INFO("THE TWIST IS: Angular: %f, with linear %f,", twist.angular.z, twist.linear.x);
            ROS_INFO_STREAM(mLoopState);
            mTwistPub.publish(twist);
            if (mLoopState == RTRSTATE::done) {
                break;
            }
        }
    }
} // namespace mrover
