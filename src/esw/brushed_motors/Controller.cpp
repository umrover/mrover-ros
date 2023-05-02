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
// EFFECTS: I2C bus, forces the angle of the controller to be a certain value
void Controller::overrideCurrentAngle(float newAngleRad) {
    auto ticks = (int32_t) (((newAngleRad) / (2 * M_PI)) * quadCPR); // convert to quad units

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
// bool Controller::isControllerLive() const {
//     return isLive;
// }

// REQUIRES: -1.0 <= input <= 1.0
// MODIFIES: currentAngle. Also makes controller live if not already.
// EFFECTS: I2C bus, Sends an open loop command scaled to PWM limits
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

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: I2C bus, returns if the MCU is calibrated
bool Controller::isCalibrated() {
    uint8_t calibration_status;

    try {
        makeLive();
        if (isControllerCalibrated) return true;

        I2C::transact(deviceAddress, motorIDRegMask | IS_CALIBRATED_OP, IS_CALIBRATED_WB,
                      IS_CALIBRATED_RB, nullptr, UINT8_POINTER_T(&calibration_status));

        isControllerCalibrated = calibration_status;

    } catch (IOFailure& e) {
        ROS_ERROR("isCalibrated failed on %s", name.c_str());
        return false;
    }

    return calibration_status;
}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: I2C bus, enables or disables limit switches
void Controller::enableLimitSwitches(bool enable) {
    if (limitAPresent) {
        enableLimitSwitch(enable, limitAEnable, motorIDRegMask | ENABLE_LIMIT_A_OP);
    }
    if (limitBPresent) {
        enableLimitSwitch(enable, limitBEnable, motorIDRegMask | ENABLE_LIMIT_B_OP);
    }
}

// REQUIRES: buffer is valid
// MODIFIES: limitEnable
// EFFECTS: I2C bus, enables limit switch if it is present
void Controller::enableLimitSwitch(bool enable, bool& limitEnable, uint8_t operation) {
    uint8_t buffer[ENABLE_LIMIT_WB];

    try {
        makeLive();

        memcpy(buffer, UINT8_POINTER_T(&limitEnable), sizeof(limitEnable));
        I2C::transact(deviceAddress, motorIDRegMask | operation, ENABLE_LIMIT_WB,
                      ENABLE_LIMIT_RB, buffer, nullptr);

        // Only set limitEnable if transaction was successful.
        limitEnable = enable;
    }
    catch (IOFailure& e) {
        ROS_ERROR("enableLimitSwitch failed on %s", name.c_str());
    }
}


// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: I2C bus, gets current absolute encoder value of MCU
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
// MODIFIES: nothing
// EFFECTS: Returns true if Controller has a (one or both) limit switch(s) is enabled.
bool Controller::getLimitSwitchEnabled() const {
    return limitAEnable || limitBEnable;
}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: I2C bus, and turns on the controller. Can be used as a way to tick the watchdog for a particular mcu.
void Controller::turnOn() const {
    try {
    I2C::transact(deviceAddress, motorIDRegMask | ON_OP, ON_WB, ON_RB,
                  nullptr, nullptr);
    } catch (IOFailure& e) {
        ROS_ERROR("turnOn failed on %s", name.c_str());
    }
}

// REQUIRES: nothing
// MODIFIES: liveMap
// EFFECTS: I2C bus, if not already live,
// configures the physical controller.
// Then makes live.

std::unordered_map<uint8_t, LiveState> Controller::liveMap;

void Controller::makeLive() {
    // if (isLive) {
    //     return;
    // }

    uint8_t key = (deviceAddress << 3) | motorID;

    // if key is absent, no motor with that key has been made live
    auto it = liveMap.find(key);
    if (it != liveMap.end()) {
        it->second.liveMutex.lock();
    } else {
        LiveState state;
        state.jointName = name;
        liveMap[key] = state;
        state.liveMutex.lock();
    }

    // already live and configured to correct motor
    if (it->second.jointName == name) {
        it->second.liveMutex.unlock();
        return;
    }

    try {
        // turn on
        I2C::transact(deviceAddress, motorIDRegMask | ON_OP, ON_WB, ON_RB,
                      nullptr, nullptr);

        uint8_t buffer[32];

        assert(motorMaxVoltage >= 0);
        assert(driverVoltage >= 0);
        auto maxPWM = (uint16_t) (100.0 * motorMaxVoltage / driverVoltage);
        assert(maxPWM <= 100);

        memcpy(buffer, UINT8_POINTER_T(&maxPWM), sizeof(maxPWM));
        I2C::transact(deviceAddress, motorIDRegMask | CONFIG_PWM_OP, CONFIG_PWM_WB,
                      CONFIG_PWM_RB, buffer, nullptr);

        memcpy(buffer, UINT8_POINTER_T(&(kP)), sizeof(kP));
        memcpy(buffer + 4, UINT8_POINTER_T(&(kI)), sizeof(kI));
        memcpy(buffer + 8, UINT8_POINTER_T(&(kD)), sizeof(kD));
        I2C::transact(deviceAddress, motorIDRegMask | CONFIG_K_OP, CONFIG_K_WB,
                      CONFIG_K_RB, buffer, nullptr);

        memcpy(buffer, UINT8_POINTER_T(&limitAPresent), sizeof(limitAPresent));
        I2C::transact(deviceAddress, motorIDRegMask | ENABLE_LIMIT_A_OP, ENABLE_LIMIT_WB,
                      ENABLE_LIMIT_RB, buffer, nullptr);

        memcpy(buffer, UINT8_POINTER_T(&limitBPresent), sizeof(limitBPresent));
        I2C::transact(deviceAddress, motorIDRegMask | ENABLE_LIMIT_B_OP, ENABLE_LIMIT_WB,
                      ENABLE_LIMIT_RB, buffer, nullptr);

        memcpy(buffer, UINT8_POINTER_T(&limitAIsActiveHigh), sizeof(limitAIsActiveHigh));
        I2C::transact(deviceAddress, motorIDRegMask | ACTIVE_LIMIT_A_OP, ACTIVE_LIMIT_WB,
                      ACTIVE_LIMIT_RB, buffer, nullptr);

        memcpy(buffer, UINT8_POINTER_T(&limitBIsActiveHigh), sizeof(limitBIsActiveHigh));
        I2C::transact(deviceAddress, motorIDRegMask | ACTIVE_LIMIT_B_OP, ACTIVE_LIMIT_WB,
                      ACTIVE_LIMIT_RB, buffer, nullptr);

        memcpy(buffer, UINT8_POINTER_T(&limitAAdjustedCounts), sizeof(limitAAdjustedCounts));
        I2C::transact(deviceAddress, motorIDRegMask | COUNTS_LIMIT_A_OP, COUNTS_LIMIT_WB,
                      COUNTS_LIMIT_RB, buffer, nullptr);

        memcpy(buffer, UINT8_POINTER_T(&limitBAdjustedCounts), sizeof(limitBAdjustedCounts));
        I2C::transact(deviceAddress, motorIDRegMask | COUNTS_LIMIT_B_OP, COUNTS_LIMIT_WB,
                      COUNTS_LIMIT_RB, buffer, nullptr);


        memcpy(buffer, UINT8_POINTER_T(&limitAIsFwd), sizeof(limitAIsFwd));
        I2C::transact(deviceAddress, motorIDRegMask | LIMIT_A_IS_FWD_OP, LIMIT_A_IS_FWD_WB,
                      LIMIT_A_IS_FWD_RB, buffer, nullptr);

        
        // update liveMap
        auto it = liveMap.find(key);
        it->second.jointName = name;
        it->second.liveMutex.unlock();

    } catch (IOFailure& e) {
        ROS_ERROR("makeLive failed on %s", name.c_str());
        throw IOFailure();
    }
}
