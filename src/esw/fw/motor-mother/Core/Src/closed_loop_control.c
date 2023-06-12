#include "closed_loop_control.h"

ClosedLoopControl *new_closed_loop_control(float _kP, float _kI, float _kD, float _kF) {
	ClosedLoopControl *control = (ClosedLoopControl *) malloc(sizeof(ClosedLoopControl));

    control->kP = _kP;
    control->kI = _kI;
    control->kD = _kD;
    control->kF = _kF;

    control->flag = 1;
    control->last_error = 0.0;
    control->cum_integ = 0.0;

    return control;
}

float calculate_pid(ClosedLoopControl *control, float target, float current) {
    if (control->flag) {
        control->last_error = target - current;
        control->flag = 0;
    }

    float error = target - current;

    control->cum_integ += error * DT;
    float diff = (error - control->last_error) / DT;

    float output = control->kP * error + control->kI * control->cum_integ + control->kD * diff + signum(error) * control->kF;

    control->last_error = error;
    return output;
}

float signum(float num) {
    if (num < 0) {
        return -1.0;
    }
    if (num > 0) {
        return 1.0;
    }
    return 0.0;
}
