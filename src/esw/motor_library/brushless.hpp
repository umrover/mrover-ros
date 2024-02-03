#pragma once

#include "params_utils.hpp"
#include <iostream>
#include <optional>
#include <unistd.h>

// TODO(quintin) this is not defined in my system can header for some reason, but moteus needs it? Is this the correct value?
#define CANFD_FDF 0x04
#include <moteus/moteus.h>

#include <can_device.hpp>
#include <controller.hpp>

namespace mrover {

    using namespace mjbots;

    enum class Mode {
        Stopped = 0,
        Fault = 1,
        PreparingToOperate1 = 2,
        PreparingToOperate2 = 3,
        PreparingToOperate3 = 4,
        PWMMode = 5,
        VoltageMode = 6,
        VoltageFOC = 7,
        VoltageDQ = 8,
        Current = 9,
        Position = 10,
        Timeout = 11,
        ZeroVelocity = 12,
        StayWithin = 13,
        MeasureInductance = 14,
        Brake = 15
    };

    enum class ErrorCode {
        DmaStreamTransferError = 1,
        DmaStreamFifiError = 2,
        UartOverrunError = 3,
        UartFramingError = 4,
        UartNoiseError = 5,
        UartBufferOverrunError = 6,
        UartParityError = 7,
        CalibrationFault = 32,
        MotorDriverFault = 33,
        OverVoltage = 34,
        EncoderFault = 35,
        MotorNotConfigured = 36,
        PwmCycleOverrun = 37,
        OverTemperature = 38,
        StartOutsideLimit = 39,
        UnderVoltage = 40,
        ConfigChanged = 41,
        ThetaInvalid = 42,
        PositionInvalid = 43,
    };

    class BrushlessController : public Controller {
    public:
        BrushlessController(ros::NodeHandle const& nh, std::string name, std::string controllerName);
        ~BrushlessController() override = default;

        void setDesiredThrottle(Percent throttle) override;
        void setDesiredVelocity(RadiansPerSecond velocity) override;
        void setDesiredPosition(Radians position) override;
        void processCANMessage(CAN::ConstPtr const& msg) override;
        double getEffort() override;
        void setStop();
        void setBrake();
        void adjust();

    private:
        moteus::Controller mController{moteus::Controller::Options{}};
        double mMeasuredEffort{};

        // Function to map throttle to velocity
        RadiansPerSecond mapThrottleToVelocity(Percent throttle);
        // Converts moteus error codes and mode codes to std::string descriptions
        static std::string moteusErrorCodeToErrorState(moteus::Mode motor_mode, ErrorCode motor_error_code);
        static std::string moteusModeToState(moteus::Mode motor_mode);
    };

} // namespace mrover
