#pragma once

#include "controller.hpp"
#include "can_manager.hpp"

#include <optional>

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
    void update(const std::vector<uint8_t> &frame) override;

    void set_desired_throttle(float throttle) override; // from -1.0 to 1.0
    void set_desired_velocity(float velocity) override; // in rev/s
    void set_desired_position(float position) override;

    BrushlessController(ros::NodeHandle& n, const std::string& name) : Controller(n, name) {
        torque = 0.3f;
    }
    ~BrushlessController() override = default;

private:
    std::optional<ErrorCode> err;
    float torque{};
};