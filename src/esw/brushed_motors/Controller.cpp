#include "Controller.h"

// REQUIRES: _name is the name of the motor,
// mcu_id is the mcu id of the controller which dictates the slave address,
// motor_id is the motor id of the motor that is to be controlled,
// motorMaxVoltage is the max allowed voltage of the motor,
// and driverVoltage is the input voltage of the driver.
// 0 < _motorMaxVoltage <= _driverVoltage <= 36.
// It is very important that the _driverVoltage is accurate,
// or else you will end up giving too much voltage to a motor,
// which is dangerous.
// MODIFIES: Controller object
// EFFECTS: Creates a virtual controller object
// that when live, will control the
// physical controller (the STM32).
Controller::Controller(
        std::string& _name,
        uint8_t mcuID,
        uint8_t _motorID,
        float _motorMaxVoltage,
        float _driverVoltage) {
    assert(0.0f < _motorMaxVoltage);
    assert(_motorMaxVoltage <= _driverVoltage);
    assert(_driverVoltage <= 36.0f);
    name = _name;
    deviceAddress = mcuID;
    motorID = _motorID;
    motorIDRegMask = motorID << 5;
    motorMaxVoltage = _motorMaxVoltage;
    driverVoltage = _driverVoltage;
    currentAngle = 0.0f;
}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Returns last saved value of angle.
// Expect a value between -M_PI and M_PI.
float Controller::getCurrentAngle() const {
    return currentAngle;
}

// REQUIRES: newAngleRad to be in radians
// MODIFIES: currentAngle
// EFFECTS: forces the angle of the controller to be a certain value
void Controller::overrideCurrentAngle(float newAngleRad) {
    int32_t ticks = (int32_t) (((newAngleRad) / (2 * M_PI)) * quadCPR); // convert to quad units

    try {
        makeLive();

        uint8_t buffer[4];
        memcpy(buffer, UINT8_POINTER_T(&ticks), sizeof(ticks));
        I2C::transact(deviceAddress, motorIDRegMask | ADJUST_OP, ADJUST_WB,
                      ADJUST_RB, buffer, nullptr);

    } catch (IOFailure& e) {
        ROS_ERROR("overrideCurrentAngle failed on %s", name.c_str());
    }
}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Returns true if Controller is live.
bool Controller::isControllerLive() const {
    return isLive;
}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Returns true if Controller is calibrated.
bool Controller::getCalibrationStatus() const {
    return isCalibrated;
}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Returns true if Controller has a (one or both) limit switch(s) is enabled.
bool Controller::getLimitSwitchEnabled() const {
    return limitAEnabled || limitBEnabled;
}

// REQUIRES: limit_polarity_request to check
// MODIFIES: nothing
// EFFECTS: Returns true if Controller has the FORWARD or REVERSE limit switch enabled.
bool Controller::getLimitSwitchEnabled(limit_polarity limit_polarity_request) const {
    if (limit_polarity_request == FORWARD) { // FORWARD limit polarity
        if (limitPolarity) return limitAEnabled;
        return limitBEnabled;
    } else { // REVERSE
        if (limitPolarity) return limitBEnabled;
        return limitAEnabled;
    }
}

// REQUIRES: -1.0 <= input <= 1.0
// MODIFIES: currentAngle. Also makes controller live if not already.
// EFFECTS: Sends an open loop command scaled to PWM limits
// based on allowed voltage of the motor. Also updates angle.
void Controller::moveOpenLoop(float input) {
    try {
        if (!(-1.0f <= input && input <= 1.0f)) {
            ROS_ERROR("moveOpenLoop on %s should only take values between -1.0 and 1.0", name.c_str());
            return;
        }

        makeLive();

        float speed = input * inversion;

        // When closing the gripper,
        // We only want to apply 10 Volts
        if (name == "HAND_GRIP") {
            float handGripClosingVoltage = 10.0f;
            float handGripClosingPercent = handGripClosingVoltage / motorMaxVoltage;
            speed = speed > handGripClosingPercent ? handGripClosingPercent : speed;
        }

        uint8_t buffer[4];
        memcpy(buffer, UINT8_POINTER_T(&speed), sizeof(speed));
        int32_t angle;

        I2C::transact(deviceAddress, motorIDRegMask | OPEN_PLUS_OP, OPEN_PLUS_WB,
                      OPEN_PLUS_RB, buffer, UINT8_POINTER_T(&angle));
        currentAngle = (float) (((float) angle / quadCPR) * 2 * M_PI);
    } catch (IOFailure& e) {
        ROS_ERROR("moveOpenLoop failed on %s", name.c_str());
    }
}

// REQUIRES: position in radians
// MODIFIES: currentAngle. Also makes controller live if not already.
// EFFECTS: Sends a closed loop command and updates angle.
void Controller::moveClosedLoop(float position) {
    try {
        if (!allowClosedLoop) {
            ROS_ERROR("moveClosedLoop on %s is not allowed", name.c_str());
            return;
        }

        if (!(closedLoopRadMin <= position && position <= closedLoopRadMax)) {
            ROS_ERROR("moveClosedLoop on %s should only take values between %f and %f", name.c_str(), closedLoopRadMin, closedLoopRadMax);
            return;
        }

        if (!isCalibrated) {
            ROS_ERROR("moveClosedLoop on %s is not allowed when not calibrated", name.c_str());
            return;
        }

        makeLive();

        float feed_forward = 0;
        uint8_t buffer[8];
        memcpy(buffer, UINT8_POINTER_T(&feed_forward), sizeof(feed_forward));
        int32_t closed_set_point = static_cast<int32_t>((position / (2.0 * M_PI)) * quadCPR);
        memcpy(buffer + 4, UINT8_POINTER_T(&closed_set_point), sizeof(closed_set_point));

        int32_t angle;

        I2C::transact(deviceAddress, motorIDRegMask | CLOSED_PLUS_OP, CLOSED_PLUS_WB,
                      CLOSED_PLUS_RB, buffer, UINT8_POINTER_T(&angle));

        currentAngle = (float) (((float) angle / quadCPR) * 2 * M_PI);
    } catch (IOFailure& e) {
        ROS_ERROR("moveOpenLoop failed on %s", name.c_str());
    }
}

// REQUIRES: nothing
// MODIFIES: isCalibrated
// EFFECTS: asks the MCU if it is calibrated
void Controller::askIsCalibrated() {
    try {
        makeLive();

        uint8_t calibration_status;
        I2C::transact(deviceAddress, motorIDRegMask | IS_CALIBRATED_OP, IS_CALIBRATED_WB,
                      IS_CALIBRATED_RB, nullptr, UINT8_POINTER_T(&calibration_status));
        
        isCalibrated = calibration_status;

    } catch (IOFailure& e) {
        ROS_ERROR("askIsCalibrated failed on %s", name.c_str());
    }
}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: gets current absolute encoder value of MCU
float Controller::getAbsoluteEncoderValue() {
    try {
        makeLive();

        float abs_enc_radians;
        I2C::transact(deviceAddress, motorIDRegMask | ABS_ENC_OP, ABS_ENC_WB,
                      ABS_ENC_RB, nullptr, UINT8_POINTER_T(&abs_enc_radians));

        return abs_enc_radians;

    } catch (IOFailure& e) {
        ROS_ERROR("getAbsoluteEncoderValue failed on %s", name.c_str());
    }

    return 0;
}

// REQUIRES: nothing
// MODIFIES: isLive
// EFFECTS: If not already live,
// configures the physical controller.
// Then makes live.
void Controller::makeLive() {
    if (isLive) {
        return;
    }

    try {
        // turn on
        I2C::transact(deviceAddress, motorIDRegMask | ON_OP, ON_WB, ON_RB,
                      nullptr, nullptr);

        uint8_t buffer[32];

        auto maxPWM = (uint16_t) (100.0 * motorMaxVoltage / driverVoltage);
        assert(0 <= maxPWM);
        assert(maxPWM <= 100);

        memcpy(buffer, UINT8_POINTER_T(&maxPWM), sizeof(maxPWM));
        I2C::transact(deviceAddress, motorIDRegMask | CONFIG_PWM_OP, CONFIG_PWM_WB,
                      CONFIG_PWM_RB, buffer, nullptr);

        memcpy(buffer, UINT8_POINTER_T(&(kP)), sizeof(kP));
        memcpy(buffer + 4, UINT8_POINTER_T(&(kI)), sizeof(kI));
        memcpy(buffer + 8, UINT8_POINTER_T(&(kD)), sizeof(kD));
        I2C::transact(deviceAddress, motorIDRegMask | CONFIG_K_OP, CONFIG_K_WB,
                      CONFIG_K_RB, buffer, nullptr);

        memcpy(buffer, UINT8_POINTER_T(&limitPolarity), sizeof(limitPolarity));
        I2C::transact(deviceAddress, motorIDRegMask | LIMIT_POLARITY_OP, LIMIT_POLARITY_WB,
                      LIMIT_POLARITY_RB, buffer, nullptr);

        memcpy(buffer, UINT8_POINTER_T(&limitAEnabled), sizeof(limitAEnabled));
        I2C::transact(deviceAddress, motorIDRegMask | CONFIG_LIMIT_A_OP, CONFIG_LIMIT_A_WB,
                      CONFIG_LIMIT_A_RB, buffer, nullptr);

        memcpy(buffer, UINT8_POINTER_T(&limitBEnabled), sizeof(limitBEnabled));
        I2C::transact(deviceAddress, motorIDRegMask | CONFIG_LIMIT_B_OP, CONFIG_LIMIT_B_WB,
                      CONFIG_LIMIT_B_RB, buffer, nullptr);

        isLive = true;

    } catch (IOFailure& e) {
        ROS_ERROR("makeLive failed on %s", name.c_str());
        throw IOFailure();
    }
}