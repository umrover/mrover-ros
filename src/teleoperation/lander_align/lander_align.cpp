#include "lander_align.hpp"
#include "lie.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <cmath>
#include <format>
#include <limits>
#include <manif/impl/se3/SE3.h>
#include <manif/impl/so3/SO3.h>
#include <optional>
#include <point.hpp>
#include <ros/init.h>
#include <ros/rate.h>
#include <unistd.h>
#include <vector>

namespace mrover {
    auto operator<<(std::ostream& ostream, RTRSTATE state) -> std::ostream& {
        return ostream << RTRSTRINGS[static_cast<std::size_t>(state)];
    }

    auto LanderAlignNodelet::onInit() -> void {
        mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle();

		//Get the values from ros 
        mZThreshold = .5;
        mXThreshold = .1;
        mPlaneOffsetScalar = 2.5;
        mDebugVectorPub = mNh.advertise<geometry_msgs::Vector3>("/lander_align/Pose", 1);
        mTwistPub = mNh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        mDebugPCPub = mNh.advertise<sensor_msgs::PointCloud2>("/lander_align/debugPC", 1);

        mActionServer.emplace(mNh, "LanderAlignAction", [&](LanderAlignGoalConstPtr const& goal) { ActionServerCallBack(goal); }, false);
        mActionServer.value().start();
        
        //TF Create the Reference Frames
        mNh.param<std::string>("camera_frame", mCameraFrameId, "zed_left_camera_frame");
        mNh.param<std::string>("world_frame", mMapFrameId, "map");

        // ROS Params for ransac
        mNh.param<double>("ransac/distance_threshold", mDistanceThreshold, 0.1);

        mPlaneLocationInWorldVector = std::make_optional<Eigen::Vector3d>(0, 0, 0);
        mNormalInWorldVector = std::make_optional<Eigen::Vector3d>(0, 0, 0);
        mLoopState = RTRSTATE::turn1;
    }

    auto LanderAlignNodelet::ActionServerCallBack(LanderAlignGoalConstPtr const goal) -> void {
        LanderAlignResult result;

        //If we haven't yet defined the point cloud we are working with
		mCloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/left/points", mNh);
		filterNormals(mCloud);
		ransac(mDistanceThreshold, 10, 100);
        if(!createSpline(7, 0.75)){
            mActionServer->setPreempted();
            return;
        }    
        publishSpline();
		//calcMotionToo();
        mPathPoints.clear();
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
            return 2 * std::numbers::pi - angle;
        }
    };

    void LanderAlignNodelet::calcMotionToo() {
        geometry_msgs::Twist twist;
        SE3d roverInWorld = SE3Conversions::fromTfTree(mTfBuffer, "base_link", "map");
        
        // Inital State
        Eigen::Vector2d initState{roverInWorld.translation().x(), roverInWorld.translation().y()};

        for (const Vector5d& point : mPathPoints) {
            double K1 = .3;
            double K2 = 1.4;
            double K3 = 1;  

            // Grab the current target state from the spline
            Eigen::Vector3d tarState{point.coeff(0, 0), point.coeff(1, 0), point.coeff(2, 0)};

            // Publish the target position in the spline
            Eigen::Matrix3d rot;
            rot <<  1, 0, 0,
                    0, 1, 0,
                    0, 0, 1;
            SE3d temp = {{point.coeff(0,0), point.coeff(1, 0), 0}, SO3d{Eigen::Quaterniond{rot}.normalized()}};
            SE3Conversions::pushToTfTree(mTfBroadcaster, "spline_point", mMapFrameId, temp);
            ROS_INFO_STREAM("Switching to target position: (x, y, theta): (" << tarState.x() << ", " << tarState.y() << ", " << tarState.z() << ")");

            double distanceToTarget = std::numeric_limits<double>::max();

            while (ros::ok() && distanceToTarget > 0.7) {
                roverInWorld = SE3Conversions::fromTfTree(mTfBuffer, "base_link", "map");
                Eigen::Vector3d xOrientation = roverInWorld.rotation().col(0); 
                double roverHeading = calcAngleWithWorldX(xOrientation);
                Eigen::Vector3d currState{roverInWorld.translation().x(), roverInWorld.translation().y(), roverHeading};

                Eigen::Vector2d distanceToTargetVector{tarState.x() - currState.x(), tarState.y() - currState.y()};
                distanceToTarget = distanceToTargetVector.norm();

                Eigen::Matrix3d rotation;
                rotation << std::cos(roverHeading),  std::sin(roverHeading), 0,
                            -std::sin(roverHeading), std::cos(roverHeading), 0,
                            0,                         0,                      1;
                
                Eigen::Vector3d errState = rotation * (currState - tarState); // maybe check error angle incase anything goes silly
                ROS_INFO_STREAM("err state " << errState.coeff(0,0) << ", " << errState.coeff(1,0) << ", " << errState.coeff(2,0));
                
                ROS_INFO_STREAM("Rover Heading " << roverHeading);

                ROS_INFO_STREAM("Term 1: " << (K2*point.coeff(3, 0)*errState.y())*pow(cos(errState.z()), 2));
                ROS_INFO_STREAM("Term 2: " << (K3*abs(point.coeff(3, 0))*tan(errState.z()))*pow(cos(errState.z()), 2));
                
                ros::Rate r(30);
                r.sleep();
                double v = (point.coeff(3, 0) - K1 * abs(point.coeff(3, 0) * (errState.x() + errState.y() * tan(errState.z()))))/(cos(errState.z()));
                double omega = point.coeff(4, 0) - ((K2*point.coeff(3, 0)*errState.y() + K3*abs(point.coeff(3, 0))*tan(errState.z()))*pow(cos(errState.z()), 2));
                ROS_INFO_STREAM("v: " << v);
                ROS_INFO_STREAM("omega: " << omega);
                ROS_INFO_STREAM("tan: " << tan(errState.z()));
                
                twist.angular.z = omega;
                twist.linear.x = v;

                if(mActionServer->isPreemptRequested()){
                    twist.angular.z = 0;
                    twist.linear.x = 0;
                    mActionServer->setPreempted();
                    mTwistPub.publish(twist);
                    break;
                }
                mTwistPub.publish(twist);
            }        
        }

        // Final Turn Adjustment
        {
            //Locations
            Eigen::Vector3d rover_dir;

            //Final msg
            geometry_msgs::Twist twist;

            //Threhsolds
            float const angular_thresh = 0.001;


            ros::Rate rate(20); // ROS Rate at 20Hzn::Matrix3d roverToPlaneNorm;
            
            while (ros::ok()) {
                // If the client has cancelled the server stop moving
                if(mActionServer->isPreemptRequested()){
                    mActionServer->setPreempted();
                    twist.angular.z = 0;
                    twist.linear.x = 0;
                    mTwistPub.publish(twist);
                    break;
                }
                
                SE3d roverInWorld = SE3Conversions::fromTfTree(mTfBuffer, "base_link", "map");
                Eigen::Vector3d roverPosInWorld{(roverInWorld.translation().x()), (roverInWorld.translation().y()), 0.0};

                Eigen::Vector3d roverToTargetForward = -mNormalInWorldVector.value();
                roverToTargetForward.normalize();

                Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
                Eigen::Vector3d left = up.cross(roverToTargetForward);

                Eigen::Matrix3d roverToTargetMat;
                roverToTargetMat.col(0) = roverToTargetForward;
                roverToTargetMat.col(1) = up;
                roverToTargetMat.col(2) = left;

                //SO3 Matrices for lie algebra
                SO3d roverToTargetSO3 = SE3Conversions::fromColumns(roverToTargetForward, left, up);
                SO3d roverSO3 = roverInWorld.asSO3();

                manif::SO3Tangentd SO3tan = roverToTargetSO3 - roverSO3; // 3 x 1 matrix of angular velocities (x,y,z)

                double angle_rate = mAngleP * SO3tan.z();
                angle_rate = (std::abs(angle_rate) > mAngleFloor) ? angle_rate : copysign(mAngleFloor, angle_rate);
                twist.angular.z = angle_rate;
                twist.linear.x = 0;

                if (std::abs(SO3tan.z()) < angular_thresh) {
                    break;
                }
                mTwistPub.publish(twist);
            }
        }
        
        twist.angular.z = 0;
        twist.linear.x = 0;
        mActionServer->setPreempted();
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
        ROS_INFO_STREAM("Filtered Points: " << mFilteredPoints.size());
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
                if (numInliers > minInliers && normal.x() != 0) { 
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

        mNormalInWorldVector = std::make_optional<Eigen::Vector3d>(zedToMap.rotation() * mNormalInZEDVector.value());

        //Calculate the SO3 in the world frame
        Eigen::Matrix3d rot;
        Eigen::Vector3d forward = mNormalInZEDVector.value().normalized();
        Eigen::Vector3d worldUp = Eigen::Vector3d::UnitZ();
        Eigen::Vector3d left = worldUp.cross(forward);
        Eigen::Vector3d up = forward.cross(left);

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

    void LanderAlignNodelet::sendTwist() {
        //Locations
        Eigen::Vector3d rover_dir;

        //Final msg
        geometry_msgs::Twist twist;

        //Threhsolds
        float const linear_thresh = 0.01; // could be member variables
        float const angular_thresh = 0.001;


        ros::Rate rate(20); // ROS Rate at 20Hzn::Matrix3d roverToPlaneNorm;
        
        while (ros::ok()) {
            // If the client has cancelled the server stop moving
            if(mActionServer->isPreemptRequested()){
                mActionServer->setPreempted();
                twist.angular.z = 0;
                twist.linear.x = 0;
                mTwistPub.publish(twist);
                break;
            }
                
            // Publish the current state of the RTR controller 
        	LanderAlignFeedback feedback;
			feedback.curr_state = RTRSTRINGS[static_cast<std::size_t>(mLoopState)];
			mActionServer->publishFeedback(feedback);
            
            // Push plane and offset locations for debugging
            SE3Conversions::pushToTfTree(mTfBroadcaster, "plane", mMapFrameId, mPlaneLocationInWorldSE3d);
            SE3Conversions::pushToTfTree(mTfBroadcaster, "offset", mMapFrameId, mOffsetLocationInWorldSE3d);
            
            SE3d roverInWorld = SE3Conversions::fromTfTree(mTfBuffer, "base_link", "map");
            Eigen::Vector3d roverPosInWorld{(roverInWorld.translation().x()), (roverInWorld.translation().y()), 0.0};
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
            SO3d roverSO3 = roverInWorld.asSO3();

            Eigen::Vector3d distanceToTargetVector = targetPosInWorld - Eigen::Vector3d{roverInWorld.translation().x(), roverInWorld.translation().y(), roverInWorld.translation().z()};

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
    

    auto LanderAlignNodelet::createSpline(int density, double offset) -> bool{
        //Constants
        const double kSplineStart = 7.0/8;
        const double dOmega = 0;
        const double dVelocity = 1;

        // Calculate the angle to the world
        const double dAngle = calcAngleWithWorldX(-mNormalInWorldVector.value());

        //Calcuulate the spline length
        ros::Duration(0.5).sleep();
        SE3d planeInRover = SE3Conversions::fromTfTree(mTfBuffer, "plane", mCameraFrameId);
        double xDistanceFromRoverToPlane = planeInRover.translation().x();
        double splineLength = kSplineStart * xDistanceFromRoverToPlane;

        if(xDistanceFromRoverToPlane <= offset/kSplineStart){
            return false;
        }
        
        // Append all of the points to each other
        // Eigen::Vector3d baseSplinePoint = mPlaneLocationInWorldVector.value() + splineLength * mNormalInWorldVector.value();
        Eigen::Vector3d baseSplinePoint = mPlaneLocationInWorldVector.value() + splineLength * mNormalInWorldVector.value();
        Eigen::Vector3d densityVector = mNormalInWorldVector.value() / density;
        Eigen::Vector3d splinePoint = Eigen::Vector3d::Zero();
        
        while(splinePoint.norm() < (splineLength - offset)){
            // Eigen::Vector3d splinePointInWorld = baseSplinePoint - splinePoint;
            Eigen::Vector3d splinePointInWorld = baseSplinePoint - splinePoint;
            // Create the new point to be added to the vector
            Vector5d newPoint;
            newPoint << splinePointInWorld.x(),
                        splinePointInWorld.y(),
                        dAngle,
                        dVelocity,
                        dOmega;
                        
            mPathPoints.emplace_back(newPoint);
            splinePoint = splinePoint + densityVector;
        }

        return true;
    }

    void LanderAlignNodelet::publishSpline(){
        Eigen::Matrix3d rot;
        rot <<  1, 0, 0,
                0, 1, 0,
                0, 0, 1;
        int index = 0;
        for(auto const & point : mPathPoints){
            SE3d mPlaneLocationInZEDSE3d = {{point.coeff(0,0), point.coeff(1,0), 0}, SO3d{Eigen::Quaterniond{rot}.normalized()}};
            SE3Conversions::pushToTfTree(mTfBroadcaster, std::format("point_{}", index), mMapFrameId, mPlaneLocationInZEDSE3d);
            index++;
        }
    }
} // namespace mrover
