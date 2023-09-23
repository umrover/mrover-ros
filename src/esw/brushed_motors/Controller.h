#pragma once

#include "I2C.h"                    // for I2C and IOFailure
#include "UART.h"                   // for UART
#include <cassert>                  // for assert
#include <cmath>                    // for M_PI
#include <cstring>                  // for string and memcpy
#include <limits>                   // for numeric limits
#include <mrover/LimitSwitchData.h> // for LimitSwitchData
#include <mutex>                    // for mutex
#include <ros/console.h>            // for ROS_ERROR

#define OFF_OP 0x00
#define OFF_WB 0
#define OFF_RB 0

#define ON_OP 0x01
#define ON_WB 0
#define ON_RB 0

#define OPEN_OP 0x02
#define OPEN_WB 4
#define OPEN_RB 0

#define OPEN_PLUS_OP 0x03
#define OPEN_PLUS_WB 4
#define OPEN_PLUS_RB 4

#define CLOSED_OP 0x04
#define CLOSED_WB 8
#define CLOSED_RB 0

#define CLOSED_PLUS_OP 0x05
#define CLOSED_PLUS_WB 8
#define CLOSED_PLUS_RB 4

#define CONFIG_PWM_OP 0x06
#define CONFIG_PWM_WB 2
#define CONFIG_PWM_RB 0

#define CONFIG_K_OP 0x07
#define CONFIG_K_WB 12
#define CONFIG_K_RB 0

#define QUAD_ENC_OP 0x08
#define QUAD_ENC_WB 0
#define QUAD_ENC_RB 4

#define ADJUST_OP 0x09
#define ADJUST_WB 4
#define ADJUST_RB 0

#define ABS_ENC_OP 0x0A
#define ABS_ENC_WB 0
#define ABS_ENC_RB 4

#define IS_CALIBRATED_OP 0x0B
#define IS_CALIBRATED_WB 0
#define IS_CALIBRATED_RB 1

#define ENABLE_LIMIT_A_OP 0x0C
#define ENABLE_LIMIT_B_OP 0x0D
#define ENABLE_LIMIT_WB 1
#define ENABLE_LIMIT_RB 0

#define ACTIVE_LIMIT_A_OP 0x0E
#define ACTIVE_LIMIT_B_OP 0x0F
#define ACTIVE_LIMIT_WB 1
#define ACTIVE_LIMIT_RB 0

#define COUNTS_LIMIT_A_OP 0x10
#define COUNTS_LIMIT_B_OP 0x11
#define COUNTS_LIMIT_WB 4
#define COUNTS_LIMIT_RB 0

#define LIMIT_A_OP 0x12
#define LIMIT_B_OP 0x13
#define LIMIT_WB 0
#define LIMIT_RB 1

#define LIMIT_A_IS_FWD_OP 0x14
#define LIMIT_A_IS_FWD_WB 1
#define LIMIT_A_IS_FWD_RB 0

#define LIMIT_DATA_OP 0x15
#define LIMIT_DATA_WB 0
#define LIMIT_DATA_RB 1

#define UINT8_POINTER_T reinterpret_cast<uint8_t*>

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

// struct LiveState {
//     bool isLive{false};
//     std::string jointName;
//     inline static std::mutex liveMutex;
// };

class Controller {
public:
    float quadCPR = std::numeric_limits<float>::infinity();
    float kP = 0.01f;
    float kI = 0.0f;
    float kD = 0.0f;
    float inversion = 1.0f;
    bool limitAPresent = false;
    bool limitBPresent = false;
    bool limitAEnable = false;
    bool limitBEnable = false;
    float calibrationVel = 0.0f;
    bool limitAIsActiveHigh = false;
    bool limitBIsActiveHigh = false;
    bool limitAIsFwd = true;
    int32_t limitAAdjustedCounts = 0;
    int32_t limitBAdjustedCounts = 0;


    // REQUIRES: _name is the name of the motor,
    // mcuID is the mcu id of the controller which dictates the slave address,
    // _motorID is the motor id of the motor that is to be controlled,
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
            uint8_t mcuID,
            uint8_t _motorID,
            float _motorMaxVoltage,
            float _driverVoltage);

    // REQUIRES: nothing
    // MODIFIES: nothing
    // EFFECTS: Returns last saved value of angle.
    // Expect a value between -M_PI and M_PI.
    float getCurrentAngle() const;

    // REQUIRES: newAngleRad to be in radians
    // MODIFIES: currentAngle
    // EFFECTS: I2C bus, forces the angle of the controller to be a certain value
    void overrideCurrentAngle(float newAngleRad);

    // REQUIRES: -1.0 <= input <= 1.0
    // MODIFIES: currentAngle. Also makes controller live if not already.
    // EFFECTS: I2C bus, Sends an open loop command scaled to PWM limits
    // based on allowed voltage of the motor. Also updates angle.
    void moveOpenLoop(float input);

    // REQUIRES: -1.0 <= input <= 1.0
    // MODIFIES: currentAngle. Also makes controller live if not already.
    // EFFECTS: UART bus, Sends an open loop command scaled to PWM limits
    // based on allowed voltage of the motor. Also updates angle.
    void moveOpenLoopViaUART(float input);

    // REQUIRES: nothing
    // MODIFIES: nothing
    // EFFECTS: I2C bus, returns if the MCU is calibrated
    bool isCalibrated();

    // REQUIRES: nothing
    // MODIFIES: nothing
    // EFFECTS: I2C bus, enables or disables limit switches
    void enableLimitSwitches(bool enable);

    // REQUIRES: nothing
    // MODIFIES: nothing
    // EFFECTS: UART bus, enables or disables limit switches
    void enableLimitSwitchesViaUART(bool enable);

    // REQUIRES: nothing
    // MODIFIES: nothing
    // EFFECTS: I2C bus, gets current absolute encoder value of MCU
    float getAbsoluteEncoderValue();

    // REQUIRES: nothing
    // MODIFIES: nothing
    // EFFECTS: Returns true if Controller has a (one or both) limit switch(s) is enabled.
    bool getLimitSwitchEnabled() const;

    // REQUIRES: nothing
    // MODIFIES: nothing
    // EFFECTS: I2C bus, and returns limit data (calibrated, limit a/b pressed).
    mrover::LimitSwitchData getLimitSwitchData();

    // REQUIRES: nothing
    // MODIFIES: nothing
    // EFFECTS: I2C bus, and turns on the controller. Can be used as a way to tick the watchdog for a particular mcu.
    void turnOn() const;

    // REQUIRES: nothing
    // MODIFIES: liveMap
    // EFFECTS: UART bus, and turns on the controller.
    // Can be used as a way to tick the watchdog for a particular mcu.
    void turnOnViaUART() const;

    // REQUIRES: nothing
    // MODIFIES: nothing
    // EFFECTS: Returns a combined ID for both the deviceAddress and motorID
    // MotorID can only be max 3 bits (0-5), and device address is max 2 bits (1 or 2)
    uint8_t combineDeviceMotorID() const;

    // REQUIRES: nothing
    // MODIFIES: liveMap
    // EFFECTS: Resets the live map. Should be only be used if needing to reset state
    // (e.g. MCU board had reset its state and needs to be reconfigured and made live)
    static void resetLiveMap();


private:
    // REQUIRES: nothing
    // MODIFIES: liveMap
    // EFFECTS: I2C bus, If not already live,
    // configures the physical controller.
    // Then makes live.
    void makeLive();

    // REQUIRES: nothing
    // MODIFIES: liveMap
    // EFFECTS: UART bus, If not already live,
    // configures the physical controller.
    // Then makes live.
    void makeLiveViaUART();

    // REQUIRES: buffer is valid and limit switch is present
    // MODIFIES: limitEnable
    // EFFECTS: I2C bus, enables limit switch if it is present
    void enableLimitSwitch(bool enable, bool& limitEnable, uint8_t operation);

    // REQUIRES: buffer is valid and limit switch is present
    // MODIFIES: limitEnable
    // EFFECTS: UART bus, enables limit switch if it is present
    void enableLimitSwitchViaUART(bool enable, bool& limitEnable, uint8_t operation);

    uint8_t deviceAddress;
    uint8_t motorID;
    uint8_t motorIDRegMask;
    float motorMaxVoltage;
    float driverVoltage;
    std::string name;

    float currentAngle;

    // key is deviceAddress and motorID (eg. if deviceAddress = 2(0b10) and motorID = 1(0b1), then key = 17(0b10001) )
    static std::unordered_map<uint8_t, std::string> liveMap;
    static std::mutex liveMapLock;

    bool isControllerCalibrated = false;

    float abs_enc_radians = 0;
    mrover::LimitSwitchData limit_switch_data;
};
