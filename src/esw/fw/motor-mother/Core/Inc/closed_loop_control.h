#ifndef INC_CLOSED_LOOP_CONTROL_H_
#define INC_CLOSED_LOOP_CONTROL_H_


#include <stdlib.h>
#include <stddef.h>
#include "stdbool.h"

#define DT 0.001f

typedef struct {

    // GAINS
    float kP;
    float kI;
    float kD;
    float kF;
    float dT;

    // MATH PLACEHOLDERS
    float last_error;
    float cum_integ;
    bool flag;

} ClosedLoopControl;

/** PUBLIC FUNCTIONS **/

ClosedLoopControl *new_closed_loop_control(float _kP, float _kI, float _kD, float _kF);

float calculate_pid(ClosedLoopControl *control, float target, float current);

/** PRIVATE FUNCTIONS MAY BE IN SOURCE FILE ONLY **/

float signum(float num);

#endif /* INC_CLOSED_LOOP_CONTROL_H_ */
