/*****************************************************************************
 *
 * pid.h
 *
 * Contains function handles, structs and macros for pid.c controller.
 *
 * Author:      Tim Menninger
 *
 *****************************************************************************/

#ifndef PIDSTRUCT
#define PIDSTRUCT

#include "common.h"

/* PID struct used to compute gains based on recent history. */
typedef struct PID {
    /* Proportion, integral and derivative gain constants (respectively) */
    double Kp;
    double Ki;
    double Kd;

    /* Setpoint is the value we are PIDing to */
    double setpoint;

    /* Values used for integral term */
    double error;
    copterTime_t t0;

    /* Values used for derivative term */
    double prevError;
    copterTime_t prevT;
} PID;

/* Initializes PID */
void initPID(PID*, double, double, double, double);
/* Puts PID in initial state */
void resetPID(PID*);
/* Updates previous values */
void updatePID(PID*, double);
/* Computes PID gain */
double computeGain(PID*, double);

#endif /* ifndef PIDSTRUCT */
