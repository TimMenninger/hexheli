/*****************************************************************************
 *
 * pid.c
 *
 * Contains functions relating to PID controlling.
 *
 * Author:      Tim Menninger
 *
 *****************************************************************************/

 #include "pid.h"

/*
 initPID

 Initializes the PID struct to match the arguments.  The values it initializes
 are the setpoint, which is the target value referred to when computing gain,
 and the gain constants for proportional, integral and derivative gains.

 Arguments:     PID *pid - The pid to initialize
                double Kp - Proportional gain constant
                double Ki - Integral gain constant
                double Kd - Derivative gain constant
                double setpoint - The setpoint that we PID to

 Returns:       None.

 Inputs:        None.

 Outputs:       None.

 Global Vars:   None.

 Revisions:     11/13/16 - Tim Menninger: Created
*/
void initPID(PID *pid, double Kp, double Ki, double Kd, double setpoint) {
    /* Ensure that we have a valid pointer */
    if (!pid)
        return;

    /* Reset the pid to its initial state */
    resetPID(pid);

    /* Initialize the pid to the values given. */
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint = setpoint;

    return;
}

/*
 resetPID

 Resets the pid to initial values.  Initial values for pid have the constants
 zeroed (which is arbitrary), and the error accumulator zeroed.  It sets the
 t0 time to the current time, and the previous values are set to the current
 values so the first compute call will have valid previous values.

 Arguments:     PID *pid - The pid to reset

 Returns:       None.

 Inputs:        None.

 Outputs:       None.

 Global Vars:   None.

 Revisions:     11/13/16 - Tim Menninger: Created
*/
void resetPID(PID *pid) {
    /* Ensure we have a valid pointer */
    if (!pid)
        return;

    /* Reset the pid by zeroing all of its values and initializing the start
       time, which is used for integral gain. */
    memset((void *) pid, 0, sizeof(PID));
    pid->t0 = getCopterTime();
    updatePID(pid, 0);

    return;
}

/*
 updatePID

 Updates the PID values.  It stores the current error and time into the slots
 designated for previous error and time, then accumulates the current error
 into the error accumulator used by the integral part of PID.

 Arguments:     PID *pid - The pid to reset
                double error - The current error

 Returns:       None.

 Inputs:        None.

 Outputs:       None.

 Global Vars:   None.

 Revisions:     11/13/16 - Tim Menninger: Created
*/
void updatePID(PID *pid, double error) {
    /* Ensure we have a valid pointer */
    if (!pid)
        return;

    /* Accumulate the current error into the pid integral error accumulator */
    pid->error += error;

    /* Update the previous values to reflect the current ones for the next
       reference of this pid */
    pid->prevError = error;
    pid->prevT = getCopterTime();

    return;
}

/*
 computeGain

 Using the values stored in the argued pid and the current process variable
 value, this computes the gain (which could be negative) that should be
 applied to the target output.  The gain is computed as:
        G(e, t) = Kp * e(t) + Ki * integral_0^t e(T)dT + Kd * de(t)/dt
 Where the parts are
        Proportional term:
            Kp * e(t)
        Integral term:
            Ki * integral_0^t e(T)dT
        Derivative term:
            Kd * de(t)/dt

 Arguments:     PID *pid - The pid to reset
                double pv - The current value that is compared to the setpoint
                    to get current error.

 Returns:       gain (double) - The computed gain.

 Inputs:        None.

 Outputs:       None.

 Global Vars:   None.

 Revisions:     11/13/16 - Tim Menninger: Created
*/
double computeGain(PID *pid, double pv) {
    /* Declare variables */
    double e, p, i, d = 0;

    /* Ensure we were given a valid pointer */
    if (!pid)
        return 0;

    /* Define the current error, e(t), as the difference between setpoint (or
       target) and argued pv (or current value) */
    e = pid->setpoint - pv;

    /* Compute the proportional gain */
    p = pid->Kp * e;

    /* Compute the integral gain */
    i = pid->Ki * (pid->error + e);

    /* Compute the derivative gain */
    if (getCopterTime() == pid->prevT)
        d = pid->Kd * (e - pid->prevError) / (getCopterTime() - pid->prevT);

    /* Update the pid's previous values for the next computation */
    updatePID(pid, e);

    return p + i + d;
}
