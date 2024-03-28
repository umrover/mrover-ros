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

    struct MoteusLimitSwitchInfo {
        bool isFwdPressed;
        bool isBwdPressed;
    };

    class BrushlessController : public Controller {
    public:
        BrushlessController(ros::NodeHandle const& nh, std::string name, std::string controllerName);
        ~BrushlessController() override = default;

        void setDesiredThrottle(Percent throttle) override;
        void setDesiredVelocity(RadiansPerSecond velocity) override;
        void setDesiredPosition(Radians position) override;
        void processCANMessage(CAN::ConstPtr const& msg) override;
        auto getEffort() -> double override;
        void setStop();
        void setBrake();
        auto getPressedLimitSwitchInfo() -> MoteusLimitSwitchInfo;
        void adjust(Radians position) override;
        void sendQuery();

    private:
        moteus::Controller mController{moteus::Controller::Options{}};
        bool limitSwitch0Present{};
        bool limitSwitch1Present{};
        bool limitSwitch0Enabled{true};
        bool limitSwitch1Enabled{true};
        bool limitSwitch0LimitsFwd{false};
        bool limitSwitch1LimitsFwd{false};
        bool limitSwitch0ActiveHigh{true};
        bool limitSwitch1ActiveHigh{true};
        bool limitSwitch0UsedForReadjustment{};
        bool limitSwitch1UsedForReadjustment{};
        Radians limitSwitch0ReadjustPosition{};
        Radians limitSwitch1ReadjustPosition{};

        double mMaxTorque{0.3};
        double mWatchdogTimeout{0.1};

        int8_t moteusAux1Info{0};
        int8_t moteusAux2Info{0};

        Radians mMinPosition, mMaxPosition;
        RadiansPerSecond mMinVelocity, mMaxVelocity;

        // Function to map throttle to velocity
        [[nodiscard]] auto mapThrottleToVelocity(Percent throttle) const -> RadiansPerSecond;
        // Converts moteus error codes and mode codes to std::string descriptions
        static auto moteusErrorCodeToErrorState(moteus::Mode motor_mode, ErrorCode motor_error_code) -> std::string;
        static auto moteusModeToState(moteus::Mode motor_mode) -> std::string;
    };

} // namespace mrover
