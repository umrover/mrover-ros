#include "Controller.h"

// REQUIRES: _name is the name of the motor,
// i2cAddress is the slave address of the physical controller,
// _motorMaxVoltage is the max allowed voltage of the motor,
// and _driverVoltage is the input voltage of the driver.
// 0 < _motorMaxVoltage <= _driverVoltage <= 36.
// It is very important that the _driverVoltage is accurate,
// or else you will end up giving too much voltage to a motor,
// which is dangerous.
// MODIFIES: Controller object
// EFFECTS: Creates a virtual controller object
// that when live, will control the
// physical controller (the STM32).
Controller::Controller(
        std::string& name,
        uint8_t i2cAddress,
        float _motorMaxVoltage,
        float _driverVoltage) {
    assert(0.0f < _motorMaxVoltage);
    assert(_motorMaxVoltage <= _driverVoltage);
    assert(_driverVoltage <= 36.0f);
    name = name;
    deviceAddress = i2cAddress;
    motorMaxVoltage = _motorMaxVoltage;
    driverVoltage = _driverVoltage;
}

// REQUIRES: nothing
// MODIFIES: currentAngle.
// EFFECTS: If controller is live,
// updates the current angle and
// returns the value in radians.
// Otherwise, do nothing.
// Expect a value between -M_PI and M_PI.
float Controller::getCurrentAngle() {
    if (!isLive) {
        return;
    }

    float return_angle = 0.0f;
    try {
        int32_t raw_angle;
        I2C::transact(deviceAddress, QUAD, nullptr, UINT8_POINTER_T(&raw_angle));

        return_angle = ((raw_angle / quadCpr) * 2.0f * M_PI) - M_PI;
    } catch (IOFailure& e) {
        printf("getCurrentAngle failed on %s\n", name.c_str());
    }
    return return_angle;
}

// REQUIRES: -M_PI <= targetAngle <= M_PI
// MODIFIES: Makes controller live if not already.
// EFFECTS: Sends a closed loop command
// to target angle in radians.
void Controller::moveClosedLoop(float targetAngle) {
    try {
        assert(-M_PI <= targetAngle);
        assert(targetAngle <= M_PI);
        makeLive();

        // The physical controller reads values from 0 - 2*M_PI.
        // Teleop sends in -M_PI to M_PI.
        targetAngle += M_PI;
        int32_t closedSetpoint = static_cast<int32_t>((targetAngle / (2.0f * M_PI)) * quadCpr);

        uint8_t buffer[32];
        memcpy(buffer + 4, UINT8_POINTER_T(&closedSetpoint), sizeof(closedSetpoint));
        I2C::transact(deviceAddress, CLOSED, buffer, nullptr);

    } catch (IOFailure& e) {
        printf("moveClosedLoop failed on %s\n", name.c_str());
    }
}

// REQUIRES: -1.0 <= input <= 1.0
// MODIFIES: Makes controller live if not already.
// EFFECTS: Sends an open loop command scaled to PWM limits
// based on allowed voltage of the motor.
void Controller::moveOpenLoop(float input) {
    try {
        assert(-1.0f <= input);
        assert(input <= 1.0f);

        makeLive();

        float speed = input * inversion;

        // When closing the gripper,
        // We only want to apply 10 Volts
        if (name == "HAND_GRIP") {
            float handGripClosingVoltage = 10;
            float handGripClosingPercent = handGripClosingVoltage / motorMaxVoltage;
            speed = speed > handGripClosingPercent ? handGripClosingPercent : speed;
        }


        uint8_t buffer[4];
        memcpy(buffer, UINT8_POINTER_T(&speed), sizeof(speed));

        I2C::transact(deviceAddress, OPEN, buffer, nullptr);
    } catch (IOFailure& e) {
        printf("moveOpenLoop failed on %s\n", name.c_str());
    }
}

// REQUIRES: nothing
// MODIFIES: Makes controller live if not already.
// EFFECTS: Zeroes the angle of the physical controller.
void Controller::zeroAngle() {
    try {
        makeLive();

        int32_t zero = 0;
        I2C::transact(deviceAddress, ADJUST, UINT8_POINTER_T(&zero), nullptr);
    } catch (IOFailure& e) {
        printf("zeroAngle failed on %s\n", name.c_str());
    }
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
        I2C::transact(deviceAddress, ON, nullptr, nullptr);

        uint8_t buffer[32];

        uint16_t maxPWM = (uint16_t) (motorMaxVoltage / driverVoltage);
        assert(0 <= maxPWM);
        assert(maxPWM <= 100);

        memcpy(buffer, UINT8_POINTER_T(&maxPWM), sizeof(maxPWM));
        I2C::transact(deviceAddress, CONFIG_PWM, buffer, nullptr);

        memcpy(buffer, UINT8_POINTER_T(&(kP)), sizeof(kP));
        memcpy(buffer + 4, UINT8_POINTER_T(&(kI)), sizeof(kI));
        memcpy(buffer + 8, UINT8_POINTER_T(&(kD)), sizeof(kD));
        I2C::transact(deviceAddress, CONFIG_K, buffer, nullptr);

        // Adjust the physical controller angle to
        // angle reported by the absolute encoder,
        // unless absolute encoder does not exist.
        float absRawAngle = M_PI;
        if (name != "ARM_B" && name != "SA_B" && name != "ARM_F") {
            I2C::transact(
                    deviceAddress,
                    ABS_ENC,
                    nullptr,
                    UINT8_POINTER_T(&(absRawAngle)));
        }

        int32_t adjustedQuad = (absRawAngle / (2.0f * M_PI)) * quadCpr;
        memcpy(buffer, UINT8_POINTER_T(&(adjustedQuad)), sizeof(adjustedQuad));
        I2C::transact(deviceAddress, ADJUST, buffer, nullptr);

        isLive = true;

    } catch (IOFailure& e) {
        printf("makeLive failed on %s\n", name.c_str());
        throw IOFailure();
    }
}