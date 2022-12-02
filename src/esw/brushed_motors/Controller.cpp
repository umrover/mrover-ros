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
        std::string& _name,
        uint8_t i2cAddress,
        float _motorMaxVoltage,
        float _driverVoltage) {
    assert(0.0f < _motorMaxVoltage);
    assert(_motorMaxVoltage <= _driverVoltage);
    assert(_driverVoltage <= 36.0f);
    name = _name;
    deviceAddress = i2cAddress;
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

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Returns true if Controller is live.
bool Controller::isControllerLive() const {
    return isLive;
}

// REQUIRES: -1.0 <= input <= 1.0
// MODIFIES: currentAngle. Also makes controller live if not already.
// EFFECTS: Sends an open loop command scaled to PWM limits
// based on allowed voltage of the motor. Also updates angle.
void Controller::moveOpenLoop(float input) {
    try {
        assert(-1.0f <= input);
        assert(input <= 1.0f);

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
        I2C::transact(deviceAddress, OPEN_PLUS, buffer, UINT8_POINTER_T(&angle));
        currentAngle = (float) ((((float) angle / quadCPR) * 2 * M_PI) - M_PI);
    } catch (IOFailure& e) {
        ROS_ERROR("moveOpenLoop failed on %s", name.c_str());
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

        auto maxPWM = (uint16_t) (100.0 * motorMaxVoltage / driverVoltage);
        assert(0 <= maxPWM);
        assert(maxPWM <= 100);

        memcpy(buffer, UINT8_POINTER_T(&maxPWM), sizeof(maxPWM));
        I2C::transact(deviceAddress, CONFIG_PWM, buffer, nullptr);

        memcpy(buffer, UINT8_POINTER_T(&(kP)), sizeof(kP));
        memcpy(buffer + 4, UINT8_POINTER_T(&(kI)), sizeof(kI));
        memcpy(buffer + 8, UINT8_POINTER_T(&(kD)), sizeof(kD));
        I2C::transact(deviceAddress, CONFIG_K, buffer, nullptr);

        isLive = true;

    } catch (IOFailure& e) {
        ROS_ERROR("makeLive failed on %s", name.c_str());
        throw IOFailure();
    }
}