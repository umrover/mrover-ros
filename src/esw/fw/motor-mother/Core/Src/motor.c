
#include "motor.h"

Motor *new_motor(bool _valid, HBridge *_hbridge, LimitSwitch *_limit_switch_a, LimitSwitch *_limit_switch_b, QuadEncoder *_encoder, AbsEncoder *_abs_encoder, ClosedLoopControl *_control) {
    Motor *motor = (Motor *) malloc(sizeof(Motor));
    motor->valid = _valid;
    motor->hbridge = _hbridge;
    motor->limit_switch_a = _limit_switch_a;
    motor->limit_switch_b = _limit_switch_b;
    motor->encoder = _encoder;
    motor->abs_encoder = _abs_encoder;

    motor->control = _control;
    motor->using_open_loop_control = 1;
    motor->output_pwm = 0;
    motor->max_pwm = 0;
    motor->desired_speed = 0;
    motor->desired_counts = 0;

    motor->is_calibrated = 0;
    motor->limit_a_is_forward = 1;
    motor->ms_since_last_commanded = 0;

    return motor;
}

void init_motor(Motor *motor, float speed) {
    init_hbridge(motor->hbridge, speed, speed);
    set_motor_speed(motor, speed);
}

void tick_motor(Motor *motor) {
	motor->ms_since_last_commanded += 1;
	if (motor->ms_since_last_commanded >= MOTORS_WATCHDOG_TIMEOUT) {
		motor->ms_since_last_commanded = 0;
		// Reset the motors
		motor->desired_speed = 0; // open loop setpoint
		motor->using_open_loop_control = 1;
	}
}

void update_motor_target(Motor *motor) {
	if (motor->using_open_loop_control) {
		set_motor_speed(motor, motor->desired_speed);
	}
	else {
        move_motor_to_target(motor);
    }
}

// at_fwd_lim = 1 means lim switch activated
void set_motor_speed(Motor *motor, float speed) {
	motor->output_pwm = speed * motor->max_pwm;
}

void update_motor_speed(Motor *motor) {
	if (!motor->hbridge->valid) {
		return;
	}
    // when speed is positive, motor goes from rev lim to fwd lim
	if (motor->limit_a_is_forward) {
		if (motor->limit_switch_a->valid &&
            motor->limit_switch_a->enabled &&
            motor->limit_switch_a->is_pressed &&
            (motor->output_pwm > 0.0f)) {
			change_hbridge_pwm(motor->hbridge, 0.0f);
		}
		else if (motor->limit_switch_b->valid &&
                 motor->limit_switch_b->enabled &&
                 motor->limit_switch_b->is_pressed &&
                 (motor->output_pwm < 0.0f)) {
			change_hbridge_pwm(motor->hbridge, 0.0f);
		}
		else {
			change_hbridge_pwm(motor->hbridge, fabsf(motor->output_pwm));
		}
		change_hbridge_dir_val(motor->hbridge, motor->output_pwm > 0.0f);
	}
	else {
		if (motor->limit_switch_a->valid && motor->limit_switch_a->is_pressed && (motor->output_pwm < 0.0f)) {
			change_hbridge_pwm(motor->hbridge, 0.0f);
		}
		else if (motor->limit_switch_b->valid && motor->limit_switch_b->is_pressed && (motor->output_pwm > 0.0f)) {
			change_hbridge_pwm(motor->hbridge, 0.0f);
		}
		else {
			change_hbridge_pwm(motor->hbridge, fabsf(motor->output_pwm));
		}
		change_hbridge_dir_val(motor->hbridge, motor->output_pwm > 0.0f);
	}

}

void move_motor_to_target(Motor *motor) {
	if (motor->is_calibrated) {
		// TODO there might be problems here
		float speed = calculate_pid(motor->control, motor->desired_counts, motor->encoder->counts);
		set_motor_speed(motor, speed);
	}
}

// Changes encoder counts and sets the calibration status
void update_motor_limit_switches(Motor *motor) {
	if (motor->limit_switch_a->valid &&
			motor->limit_switch_a->enabled &&
			motor->limit_switch_a->is_pressed) {
		motor->encoder->counts = motor->limit_switch_a->associated_count;
		motor->is_calibrated = 1;
	} else if (motor->limit_switch_b->valid &&
			motor->limit_switch_b->enabled &&
			motor->limit_switch_b->is_pressed) {
		motor->encoder->counts = motor->limit_switch_b->associated_count;
		motor->is_calibrated = 1;
	}

}

void refresh_motor_absolute_encoder_value(Motor *motor) {
	if (motor->abs_encoder->valid) {
		refresh_angle_radians(motor->abs_encoder);
	}
}

