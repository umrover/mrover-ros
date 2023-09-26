#pragma once

#include "controller.hpp"

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

class BrushlessController final : public Controller {
public:
    void update(uint64_t frame) override;

    void set_desired_speed_unit(double speed) override; // from -1.0 to 1.0
    void set_desired_speed_rev_s(double speed);  // in rev/s
    void set_desired_position(int position) override;
    MotorType get_type() override;

private:
    std::optional<ErrorCode> err;
    float torque{};
};