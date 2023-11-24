#pragma once

#include <string>

#include <ros/ros.h>

#include <can_device.hpp>
#include <chrono>
#include <iostream>
#include <units/units.hpp>

namespace mrover {

    class Controller {
    public:
        Controller(ros::NodeHandle const& nh, std::string name, std::string controllerName)
            : mNh{nh},
              mName{std::move(name)},
              mControllerName{std::move(controllerName)},
              mDevice{nh, mName, mControllerName},
              mIncomingCANSub{mNh.subscribe<CAN>(std::format("can/{}/in", mControllerName), 16, &Controller::processCANMessage, this)} {
            updateLastConnection();
            mHeartbeatTimer = mNh.createTimer(ros::Duration(0.1), &Controller::heartbeatCallback, this);
        }

        virtual ~Controller() = default;

        virtual void setDesiredThrottle(Percent throttle) = 0;          // from -1.0 to 1.0
        virtual void setDesiredVelocity(RadiansPerSecond velocity) = 0; // joint output
        virtual void setDesiredPosition(Radians position) = 0;          // joint output
        virtual void processCANMessage(CAN::ConstPtr const& msg) = 0;
        Radians getCurrentPosition() { return mCurrentPosition; }
        RadiansPerSecond getCurrentVelocity() { return mCurrentVelocity; }
        std::string getErrorState() { return mErrorState; }
        std::string getState() { return mState; }
        virtual double getEffort() = 0;
        bool isLimitHit(uint8_t index) {
            return mLimitHit.at(index);
        }
        void updateLastConnection() {
            mLastConnection = std::chrono::high_resolution_clock::now();
        }

        void heartbeatCallback(ros::TimerEvent const&) {
            auto duration = std::chrono::high_resolution_clock::now() - mLastConnection;
            if (duration < std::chrono::milliseconds(100)) {
                setDesiredThrottle(0_percent);
            }
        }


    protected:
        ros::NodeHandle mNh;
        std::string mName, mControllerName;
        CanDevice mDevice;
        ros::Subscriber mIncomingCANSub;
        RadiansPerSecond mMinVelocity{}, mMaxVelocity{};
        Radians mMinPosition{}, mMaxPosition{};
        Radians mCurrentPosition{};
        RadiansPerSecond mCurrentVelocity{};
        bool mIsCalibrated{};
        std::string mErrorState;
        std::string mState;
        std::array<bool, 4> mLimitHit{};
        std::chrono::high_resolution_clock::time_point mLastConnection;
        ros::Timer mHeartbeatTimer;
    };

} // namespace mrover
