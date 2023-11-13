#pragma once

#include <string>

#include <ros/ros.h>

#include <can_device.hpp>
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
        }

        virtual ~Controller() = default;

        virtual void setDesiredThrottle(Percent throttle) = 0;          // from -1.0 to 1.0
        virtual void setDesiredVelocity(RadiansPerSecond velocity) = 0; // joint output
        virtual void setDesiredPosition(Radians position) = 0;          // joint output
        virtual void processCANMessage(CAN::ConstPtr const& msg) = 0;
        Radians getCurrentPosition() { return mCurrentPosition; }
        RadiansPerSecond getCurrentVelocity() { return mCurrentVelocity; }
        std::string getErrorState() { return mErrorState; }
        std::string getState() { return mState; }  // TODO - we probably don't need both mErrorState and mState
        virtual double getEffort() = 0;
        bool isLimitHit(uint8_t index) {
            return mLimitHit.at(index);
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
    };

} // namespace mrover
