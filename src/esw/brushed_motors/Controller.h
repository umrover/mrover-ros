#pragma once

#include "I2C.h"         // for I2C and IOFailure
#include <assert.h>      // for assert
#include <cmath>         // for M_PI
#include <limits>        // for numeric limits
#include <mutex>         // for mutex
#include <ros/console.h> // for ROS_INFO
#include <string.h>      // for string and memcpy

#define OFF 0x00, 0, 0
#define ON 0x0F, 0, 0
#define OPEN 0x10, 4, 0
#define OPEN_PLUS 0x1F, 4, 4
#define CLOSED 0x20, 8, 0
#define CLOSED_PLUS 0x2F, 8, 4
#define CONFIG_PWM 0x30, 2, 0
#define CONFIG_K 0x3F, 12, 0
#define QUAD 0x40, 0, 4
#define ADJUST 0x4F, 4, 0
#define ABS_ENC 0x50, 0, 4
#define LIMIT 0x60, 0, 1
#define CALIBRATED 0x6F, 0, 1
#define LIMIT_ON 0x7F, 1, 0
#define UINT8_POINTER_T reinterpret_cast<uint8_t*>

#define CALIBRATED_BOOL 0xFF

/*
Virtual Controllers store information about various
controller-specific parameters (such as encoder cpr).
The virtual Controller class also has functions representing
the possible transactions that can be had with the physical controller.
The virtual Controller will not attempt to communicate with its
physical controller unless "activated" by an appropriate ROS
message relayed by ROSHandler.h.
(e.g. A virtual RA Controller will never attempt to communicate
with its physical RA controller unless an RA-related ROS message is
sent. This is to prevent multiple virtual Controller objects from
trying to contact the same physical Controller object.)
*/


class Controller {
public:
    float quadCPR = std::numeric_limits<float>::infinity();
    float kP = 0.01f;
    float kI = 0.0f;
    float kD = 0.0f;
    float inversion = 1.0f;

    // REQUIRES: _name is the name of the motor,
    // i2cAddress is the slave address of the physical controller,
    // motorMaxVoltage is the max allowed voltage of the motor,
    // and driverVoltage is the input voltage of the driver.
    // 0 < motorMaxVoltage <= driverVoltage <= 36.
    // It is very important that the driverVoltage is 100% accurate,
    // or else you will end up giving too much voltage to a motor,
    // which is dangerous.
    // MODIFIES: Controller object
    // EFFECTS: Creates a virtual controller object
    // that when live, will control the
    // physical controller (the STM32).
    Controller(
            std::string& _name,
            uint8_t i2cAddress,
            float _motorMaxVoltage,
            float _driverVoltage);

    // REQUIRES: nothing
    // MODIFIES: nothing
    // EFFECTS: Returns true if Controller is live.
    bool isControllerLive() const;

    // REQUIRES: nothing
    // MODIFIES: nothing
    // EFFECTS: Returns last saved value of angle.
    // Expect a value between -M_PI and M_PI.
    float getCurrentAngle() const;

    // REQUIRES: -1.0 <= input <= 1.0
    // MODIFIES: currentAngle. Also makes controller live if not already.
    // EFFECTS: Sends an open loop command scaled to PWM limits
    // based on allowed voltage of the motor. Also updates angle.
    void moveOpenLoop(float input);

private:
    // REQUIRES: nothing
    // MODIFIES: isLive
    // EFFECTS: If not already live,
    // configures the physical controller.
    // Then makes live.
    void makeLive();

    uint8_t deviceAddress;
    float motorMaxVoltage;
    float driverVoltage;
    std::string name;

    float currentAngle;

    bool isLive = false;
};
