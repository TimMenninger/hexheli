/*****************************************************************************
 *
 * copter.h
 *
 * Function definitionsand macros for copter.c
 *
 * Author:      Tim Menninger
 *
 *****************************************************************************/

#ifndef COPTER
#define COPTER

#include "common.h"
#include "position.h"

/* Tells us if we are in automode */
#define AUTOMODE                true
/* Motor directions */
#define MOTOR_HEX0_CLKWISE      true
#define MOTOR_TOP_CLKWISE       true
/* PID gain multipliers */
#define PID_TILT_SCALE          1
#define PID_LIFT_SCALE          1

typedef struct Motor {
    char pwm;       /* 0-255 PWM value */
    bool clockwise; /* Defines rotation direction. */
} Motor;

/* Copter time access functions */
void setCopterTime(copterTime_t);
void updateCopterTime(copterTime_t);
copterTime_t getCopterTime();
/* Copter position and orientation access functions */
void setTargetQuaternion(Quaternion);
Quaternion getTargetQuaternion();
/* Copter speed and direction */
void setTargetCopterDir(angle_t);
angle_t getTargetCopterDir();
void setTargetCopterSpeed(speed_t);
speed_t getTargetCopterSpeed();

/* Changes speed of motors in attempt to stabilize the copter */
void stabilize();
/* Sets speed without changing direction */
void setSpeed(speed_t speed);
/* Sets direction without changing speed */
void setDirection(angle_t degrees);
/* Rotates relative to current direction */
void setRelativeRotate(angle_t degrees);
/* Moves laterally. Direction absolute */
void setMoveLateral(speed_t speed, angle_t dir);
/* Moves up or down */
void setMoveVertical(bool upward, speed_t speed);
/* Initializes the eight propeller motors */
void initMotors();

#endif /* ifndef COPTER */
