#pragma once

#include <string>

#include <ros/ros.h>

#include <can_device.hpp>
#include <chrono>
#include <iostream>
#include <units/units.hpp>

#include <mrover/AdjustMotor.h>
#include <mrover/ControllerState.h>
#include <mrover/Position.h>
#include <mrover/Throttle.h>
#include <mrover/Velocity.h>
#include <sensor_msgs/JointState.h>

namespace mrover {

    class Controller {
    public:
        Controller(ros::NodeHandle const& nh, std::string name, std::string controllerName)
            : mNh{nh},
              mName{std::move(name)},
              mControllerName{std::move(controllerName)},
              mDevice{nh, mName, mControllerName},
              mIncomingCANSub{
                      mNh.subscribe<CAN>(
                              std::format("can/{}/in", mControllerName), 16, &Controller::processCANMessage, this)} {
            // Subscribe to the ROS topic for commands
            mMoveThrottleSub = mNh.subscribe<Throttle>(std::format("{}_throttle_cmd", mControllerName), 1, &Controller::moveMotorsThrottle, this);
            mMoveVelocitySub = mNh.subscribe<Velocity>(std::format("{}_velocity_cmd", mControllerName), 1, &Controller::moveMotorsVelocity, this);
            mMovePositionSub = mNh.subscribe<Position>(std::format("{}_position_cmd", mControllerName), 1, &Controller::moveMotorsPosition, this);

            mJointDataPub = mNh.advertise<sensor_msgs::JointState>(std::format("{}_joint_data", mControllerName), 1);
            mControllerDataPub = mNh.advertise<ControllerState>(std::format("{}_controller_data", mControllerName), 1);

            mPublishDataTimer = mNh.createTimer(ros::Duration(0.1), &Controller::publishDataCallback, this);

            mAdjustServer = mNh.advertiseService(std::format("{}_adjust", mControllerName), &Controller::adjustServiceCallback, this);
        }

        virtual ~Controller() = default;

        virtual void setDesiredThrottle(Percent throttle) = 0;          // from -1.0 to 1.0
        virtual void setDesiredVelocity(RadiansPerSecond velocity) = 0; // joint output
        virtual void setDesiredPosition(Radians position) = 0;          // joint output
        virtual void processCANMessage(CAN::ConstPtr const& msg) = 0;
        virtual double getEffort() = 0;
        virtual void adjust(Radians position) = 0;

        void moveMotorsThrottle(Throttle::ConstPtr const& msg) {
            if (msg->names.size() != 1 || msg->names.at(0) != mControllerName || msg->throttles.size() != 1) {
                ROS_ERROR("Throttle request at topic for %s ignored!", msg->names.at(0).c_str());
                return;
            }

            setDesiredThrottle(msg->throttles.at(0));
        }


        void moveMotorsVelocity(Velocity::ConstPtr const& msg) {
            if (msg->names.size() != 1 || msg->names.at(0) != mControllerName || msg->velocities.size() != 1) {
                ROS_ERROR("Velocity request at topic for %s ignored!", msg->names.at(0).c_str());
                return;
            }

            setDesiredVelocity(RadiansPerSecond{msg->velocities.at(0)});
        }

        void moveMotorsPosition(Position::ConstPtr const& msg) {
            if (msg->names.size() != 1 || msg->names.at(0) != mControllerName || msg->positions.size() != 1) {
                ROS_ERROR("Position request at topic for %s ignored!", msg->names.at(0).c_str());
                return;
            }

            setDesiredPosition(Radians{msg->positions.at(0)});
        }

        auto publishDataCallback(ros::TimerEvent const&) -> void {
            {
                sensor_msgs::JointState jointState;
                jointState.header.stamp = ros::Time::now();
                jointState.name = {mControllerName};
                jointState.position = {mCurrentPosition.get()};
                jointState.velocity = {mCurrentVelocity.get()};
                jointState.effort = {getEffort()};
                mJointDataPub.publish(jointState);
            }
            {
                ControllerState controllerState;
                controllerState.name = {mControllerName};
                controllerState.state = {mState};
                controllerState.error = {mErrorState};
                std::uint8_t limit_hit{};
                for (int i = 0; i < 4; ++i) {
                    limit_hit |= mLimitHit.at(i) << i;
                }
                controllerState.limit_hit = {limit_hit};

                mControllerDataPub.publish(controllerState);
            }
        }

        bool adjustServiceCallback(AdjustMotor::Request& req, AdjustMotor::Response& res) {
            if (req.name != mControllerName) {
                ROS_ERROR("Adjust request at server for %s ignored", req.name.c_str());
                res.success = false;
                return true;
            }
            adjust(Radians{req.value});
            res.success = true;
            return true;
        }

    protected:
        Dimensionless mVelocityMultiplier;
        ros::NodeHandle mNh;
        std::string mName, mControllerName;
        CanDevice mDevice;
        ros::Subscriber mIncomingCANSub;
        Radians mCurrentPosition{};
        RadiansPerSecond mCurrentVelocity{};
        Percent mCalibrationThrottle{};
        bool mIsCalibrated{};
        bool mhasLimit{};
        std::string mErrorState;
        std::string mState;
        std::array<bool, 4> mLimitHit{};

        ros::Subscriber mMoveThrottleSub;
        ros::Subscriber mMoveVelocitySub;
        ros::Subscriber mMovePositionSub;
        ros::Publisher mJointDataPub;
        ros::Publisher mControllerDataPub;
        ros::Timer mPublishDataTimer;

        ros::ServiceServer mAdjustServer;
    };

} // namespace mrover
