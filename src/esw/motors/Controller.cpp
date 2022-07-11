#include "Controller.h"

// If this Controller is not live, make it live by configuring the real controller
void Controller::make_live() {
    if (ControllerMap::check_if_live(name)) {
        return;
    }

    try {
        // turn on
        transact(ON, nullptr, nullptr);

        uint8_t buffer[32];
        // buffer sends max percentage speed
        memcpy(buffer, UINT8_POINTER_T(&speed_max), sizeof(speed_max);
        transact(CONFIG_PWM, buffer, nullptr);

        // config kp, ki, pd
        memcpy(buffer, UINT8_POINTER_T(&(kP)), sizeof(kP));
        memcpy(buffer + 4, UINT8_POINTER_T(&(kI)), sizeof(kI));
        memcpy(buffer + 8, UINT8_POINTER_T(&(kD)), sizeof(kD));
        transact(CONFIG_K, buffer, nullptr);

        // get absolute encoder correction #
        // not needed for joint F

        float abs_raw_angle = 0.0f;

        if (name == "ARM_B" || name == "SA_B" || name == "ARM_F") {
            abs_raw_angle = M_PI;
        } else {
            transact(ABS_ENC, nullptr, UINT8_POINTER_T(&(abs_raw_angle)));
        }

        // get value in quad counts adjust quadrature encoder
        int32_t adjusted_quad = (abs_raw_angle / (2.0f * M_PI)) * quad_cpr;
        memcpy(buffer, UINT8_POINTER_T(&(adjusted_quad)), sizeof(adjusted_quad));
        transact(ADJUST, buffer, nullptr);

        ControllerMap::make_live(name);

        // TODO - UNCOMMENT ONCE LIMIT SWITCHES ARE IMPLEMENTED
        /*
        if (name == "ARM_B" || name == "SA_B")
        {

            calibrate_joint();
        }
        */
    } catch (IOFailure& e) {
        printf("activate failed on %s\n", name.c_str());
        throw IOFailure();
    }
}

// Helper function to convert raw angle to radians. Also checks if new angle is close to old angle
void Controller::record_angle(int32_t raw_angle) {
    // record quadrature
    current_angle_m.lock();
    current_angle = ((raw_angle / quad_cpr) * 2.0f * M_PI) - M_PI;
    current_angle_m.unlock();
}

// Wrapper for I2C transact, autofilling the i2c address of the Controller by using ControllerMap::get_i2c_address()
void Controller::transact(uint8_t cmd, uint8_t write_num, uint8_t read_num, uint8_t* write_buf, uint8_t* read_buf) {
    I2C::transact(ControllerMap::get_i2c_address(name), cmd, write_num, read_num, write_buf, read_buf);
}

// Initialize the Controller. Need to know which nucleo and which channel on the nucleo to use
Controller::Controller(std::string name, uint16_t pwm_max) : name(name), speed_max(pwm_max) {}

// Calibrate joint -- should only be used for joint b
void Controller::calibrate_joint() {
    if (!ControllerMap::check_if_live(name)) {
        return;
    }

    try {
        if (name != "ARM_B" && name != "SA_B") {
            printf("calibration not supported on %s\n", name.c_str());
            return;
        }

        do {
            refresh_calibration_data();
            move_open_loop(0.3);
        } while (!calibrated);
    } catch (IOFailure& e) {
        printf("calibrate joint failed on %s\n", name.c_str());
    }
}

// Sends a config command with PID inputs
void Controller::config_pid(float KP, float KI, float KD) {
    for (int attempts = 0; attempts < 100; ++attempts) {
        try {
            make_live();

            uint8_t buffer[32];
            memcpy(buffer, UINT8_POINTER_T(&KP), sizeof(KP));
            memcpy(buffer + 4, UINT8_POINTER_T(&KI), sizeof(KI));
            memcpy(buffer + 8, UINT8_POINTER_T(&KD), sizeof(KD));
            transact(CONFIG_K, buffer, nullptr);
        } catch (IOFailure& e) {
            printf("config failed on %s\n", name.c_str());
        }
    }
}

// Sends a limit switch enable command
void Controller::enable_limit_switch(bool enable) {
    if (!ControllerMap::check_if_live(name)) {
        return;
    }
    try {
        int8_t limit_enabled = enable ? 0xFF : 0x00;
        transact(LIMIT_ON, nullptr, UINT8_POINTER_T(&limit_enabled));
    } catch (IOFailure& e) {
        printf("limit switch enable failed on %s\n", name.c_str());
    }
}

// Gets current angle (safely)
float Controller::get_current_angle() {
    float return_angle = 0.0f;
    current_angle_m.lock();
    return_angle = current_angle;
    current_angle_m.unlock();
    return return_angle;
}

// Sends a closed loop command with target angle in radians and optional precalculated torque in Nm
void Controller::move_closed_loop(float torque, float target) {
    try {
        make_live();
        // TODO - UNCOMMENT ONCE LIMIT SWITCHES ARE IMPLEMENTED
        /*
        refresh_calibration_data();
        if ((name == "ARM_B" || name == "SA_B") && !calibrated)
        {
            printf("move_closed_loop failed because joint B is not yet calibrated");
            return;
        }
        */
        // TODO - INVESTIGATE IF parameter torque IS NEEDED. if not, then we should just get rid
        // of it and simplify
        // the function to move_closed_loop(float target).
        float feed_forward = 0.0f; // torque * torque_scale;
        uint8_t buffer[32];
        int32_t angle;
        memcpy(buffer, UINT8_POINTER_T(&feed_forward), sizeof(feed_forward));

        // we read values from 0 - 2pi, teleop sends in -pi to pi
        target += M_PI;

        int32_t closed_setpoint = static_cast<int32_t>((target / (2.0f * M_PI)) * quad_cpr);
        memcpy(buffer + 4, UINT8_POINTER_T(&closed_setpoint), sizeof(closed_setpoint));

        transact(CLOSED_PLUS, buffer, UINT8_POINTER_T(&angle));

        // handles if is joint B
        record_angle(angle);
    } catch (IOFailure& e) {
        printf("closed loop failed on %s\n", name.c_str());
    }
}

// Handles an open loop command with input [-1.0, 1.0], scaled to PWM limits
void Controller::move_open_loop(float input) {
    try {
        make_live();

        // TODO - UNCOMMENT ONCE LIMIT SWITCHES ARE IMPLEMENTED
        /*
        refresh_calibration_data();
        if ((name == "ARM_B" || name == "SA_B") && !calibrated)
        {
            printf("move_open_loop failed because joint B is not yet calibrated");
            return;
        }
        */

        uint8_t buffer[4];
        float speed = throttle(input) * inversion;

        // When closing the gripper, we want 0.84 * 12 V = 10 V.
        // Closing is currently when speed is positive
        // TODO - make this more configurable for what we want closing
        // gripper voltage to be.
        if (name == "HAND_GRIP") {
            speed = speed > 0.84f ? 0.84f : speed;
        }


        memcpy(buffer, UINT8_POINTER_T(&speed), sizeof(speed));

        int32_t raw_angle;

        transact(OPEN_PLUS, buffer, UINT8_POINTER_T(&raw_angle));

        record_angle(raw_angle);
    } catch (IOFailure& e) {
        printf("open loop failed on %s\n", name.c_str());
    }
}

// Sends a get angle command
void Controller::refresh_calibration_data() {
    if (!ControllerMap::check_if_live(name)) {
        return;
    }

    try {
        uint8_t calibration_state;

        transact(CALIBRATED, nullptr, UINT8_POINTER_T(&calibration_state));

        // calibration_state is either 0xFF or 0x00.
        // 0xFF means it is calibrated, 0x00 means not calibrated.
        calibrated = calibration_state == CALIBRATED_BOOL;
    } catch (IOFailure& e) {
        printf("calibration data failed on %s\n", name.c_str());
    }
}

// Sends a get angle command
void Controller::refresh_quad_angle() {
    if (!ControllerMap::check_if_live(name)) {
        return;
    }

    try {
        int32_t raw_angle;
        transact(QUAD, nullptr, UINT8_POINTER_T(&raw_angle));

        record_angle(raw_angle);
    } catch (IOFailure& e) {
        printf("angle failed on %s\n", name.c_str());
    }
}

// Limits throttle
float Controller::throttle(float input) {
    if (input > 1.0f) {
        input = 1.0f;
    } else if (input < -1.0f) {
        input = -1.0f;
    }
    return input;
}

// Sends a zero command
void Controller::zero() {
    try {
        make_live();

        int32_t zero = 0;
        transact(ADJUST, UINT8_POINTER_T(&zero), nullptr);
    } catch (IOFailure& e) {
        printf("zero failed on %s\n", name.c_str());
    }
}
