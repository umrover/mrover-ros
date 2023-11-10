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

        // TODO: receive information

        virtual void setDesiredThrottle(Percent throttle) = 0;          // from -1.0 to 1.0
        virtual void setDesiredVelocity(RadiansPerSecond velocity) = 0; // joint output
        virtual void setDesiredPosition(Radians position) = 0;          // joint output
        virtual void processCANMessage(CAN::ConstPtr const& msg) = 0;
        Radians getCurrentPosition() { return mCurrentPosition; }
        RadiansPerSecond getCurrentVelocity() { return mCurrentVelocity; }
        std::string getErrorState() { return mErrorState; }
        std::string getState() { return mState; }
        virtual double getEffort() = 0; // TODO implement in base files. for brushed it is 0/nan, for brushless it exists.

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
        bool mLimitAHit{};
        bool mLimitBHit{};
        bool mLimitCHit{};
        bool mLimitDHit{};
        //    virtual void send_CAN_frame(uint64_t frame) = 0;
    };

} // namespace mrover
