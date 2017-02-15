/*****************************************************************************
 *
 * copter.c
 *
 * Contains code for controlling the copter and its motors.
 * The assumed orientation is the copter is such that moving forward should
 * affect the speeds of four of the six arms, and moving sideways affects
 * all six.  In otherwords, moving sideways would move in the direction of
 * a point of the hexagon formed by the six propellers.  Moving forward in
 * this orientation would be moving in the +x direction, rotated 0 degrees.
 * Motor 0 will be the one whose arm makes the smallest angle with the +x
 * axis, and will count upward in increasing angles.
 *
 * This also uses the idea of autoMode, which is used to determine whether
 * we are controlling position in software or if someone is manually dictating
 * how to move (so all that is needed then is stability).
 *
 * Author:      Tim Menninger
 *
 *****************************************************************************/

 #include "copter.h"

/******************************************************************************

 GLOBAL VARIABLES AND ACCESS FUNCTIONS

******************************************************************************/

/* Tells us if we are running autonomously, which is used in stabilization */
bool         autoMode;

/* Keeps track of the time in milliseconds.  This will be referred to as
   copter time, or the time since the copter started running. */
copterTime_t copterTime;
void         setCopterTime(copterTime_t time)    { copterTime = time; }
copterTime_t getCopterTime()                     { return copterTime; }

/* Depending on the direction of travel and speed, this rotation matrix will
   describe the rotation that the copter is to be undergoing. */
Quaternion   targetQuaternion;
void         setTargetQuaternion(Quaternion q)   { targetQuaternion = q; }
Quaternion   getTargetQuaternion()               { return targetQuaternion; }

/* Keep track of the direction we want the copter facing.  This will allow us
   to change speeds without re-computing the direction. */
angle_t      targetCopterDir;
void         setTargetCopterDir(angle_t dir)     { targetCopterDir = dir; }
angle_t      getTargetCopterDir()                { return targetCopterDir; }

/* Keep track of the speed of the copter so we can change the direction
   more easily. */
speed_t      targetCopterSpeed;
void         setTargetCopterSpeed(speed_t speed) { targetCopterSpeed = speed; }
speed_t      getTargetCopterSpeed()              { return targetCopterSpeed; }

/* Keep track of the target X, Y, Z position of the copter. */
Point3d      targetPosition;
void         setTargetPosition(Point3d p)        { targetPosition = p; }
Point3d      getTargetPosition()                 { return targetPosition; }

/* The six hexcopter motors and the two coaxial motors */
/* i'th motor is at i*60 degrees clockwise from front */
Motor        hexMotors[6];
Motor        topHeliMotor;
Motor        botHeliMotor;
void         setHexMotor(int i, Motor m)         { hexMotors[i] = m; }
void         setTopHeliMotor(Motor m)            { topHeliMotor = m; }
void         setBotHeliMotor(Motor m)            { botHeliMotor = m; }
Motor        getHexMotor(int i)                  { return hexMotors[i]; }
Motor        getTopHeliMotor()                   { return topHeliMotor; }
Motor        getBotHeliMotor()                   { return botHeliMotor; }

/******************************************************************************

 COPTER FUNCTIONS

******************************************************************************/

/*
 stabilize

 Use target and current quaternions to stabilize the copter.  We will do this
 by finding the normal of the target and current copter positions from the
 respective quaternions.  From this, we can increase and decrease motor speeds
 to try pushing those to be the same.  We can find normals by applying
 quaternions to {0, 0, 1}.  We then apply the inverse of that rotation to the
 quaternions so that the system we see them in has the normal straight up.
 From here, we find the rotation by doing a similar process applying the
 quaternion to {1, 0, 0}, then taking atan(y/x) to find the two rotation
 angles.  We change motor speeds accordingly to rotate to that angle.

 Arguments:     bool autoMode - If true, this loop should try to control where
                    the copter is vertically.  If false, it is assumed that
                    a human is controlling it, the interface being with the
                    setMoveVertical function

 Returns:       None.

 Inputs:        None.

 Outputs:       Motors - The motors are written to in order to stabilize

 Global Vars:

 Revisions:     12/04/16 - Tim Menninger: Uses quaternions to rotate
                01/11/17 - Tim Mennigner: Changed to compute normal of copter
                01/17/17 - Tim Menninger: Fixed bugs, added comments regarding
                    how to handle tilting
*/
void stabilize() {
    /* Variables used */
    Point3d copterNormal = {0, 0, 1};
    Point3d targetNormal = {0, 0, 1};
    Quaternion copter = getCopterQuaternion();
    Quaternion target = getTargetQuaternion();
    Quaternion undoQuat, temp;
    Point3d undoVec, rotationVec = {1, 0, 0};
    angle_t undoAngle;
    double mag, copterRot, targetRot, undoRot;

    /* Apply target and current quaternions to {0, 0, 1} to get normals */
    applyQuaternion(copter, &copterNormal);
    applyQuaternion(target, &targetNormal);

    /* Look at x and y components to alter rotor speeds, either increasing
       or decreasing them to change the tilt in either direction.  Changes
       in the x direction will affect the motors 0, 2, 3 and 5 equally, with
       the effects of 2 and 3 being equal but opposite to 0 and 5. Then,
       changes in the y direction will affect motors 1 and 4 half as much
       as motors 0, 2, 3 and 5, with 0, 1 and 2 changes being opposite in
       sign to 3, 4 and 5.  These are related as such: if we tilt some
       angle in the y direction, we change motors 0, 2, 3 and 5 by 2a, and
       motors 1 and 4 by a.  To achieve the same tilt in the x direction,
       we change motors 0, 2, 3 and 5 each by 4a/sqrt(3). */
    /* TODO: use PID and set six arms to change tilt */

    /* Using the normals, we can "undo" the rotation so that the system we
       see the copter in has the normals as 0, 0, 1 in both cases.  Now, it
       is easy to see how much we should rotate.  In both cases, apply the
       new quaternion to {1, 0, 0} and compute atan(y/x).  We now have two
       angles and can rotate the copter to get to the target rotation. */
    undoVec.x = copterNormal.y; /* Filling from cross product */
    undoVec.y = -1 * copterNormal.x;
    undoVec.z = 0;
    mag = sqrt(undoVec.x*undoVec.x + undoVec.y*undoVec.y);
    undoAngle = atan(mag / sqrt(1 - mag*mag));
    temp = copter;
    createQuaternion(&undoQuat, undoVec, undoAngle);
    multiplyQuaternions(undoQuat, temp, &copter);
    applyQuaternion(copter, &rotationVec);
    /* Can now compute copter rotation about its normal */
    copterRot = atan(rotationVec.y / rotationVec.x);
    copterRot = (x < 0 ? -1 * copterRot : copterRot);
    /* And now undo for target quaternion */
    undoVec.x = targetNormal.y; /* Filling from cross product */
    undoVec.y = -1 * targetNormal.x;
    undoVec.z = 0;
    mag = sqrt(undoVec.x*undoVec.x + undoVec.y*undoVec.y);
    undoAngle = atan(mag / sqrt(1 - mag*mag));
    temp = target;
    createQuaternion(&undoQuat, undoVec, undoAngle);
    multiplyQuaternions(undoQuat, temp, &target);
    rotationVec.x = 1; /* Reset rotation vector to be x axis */
    rotationVec.y = 0;
    rotationVec.z = 0;
    applyQuaternion(copter, &rotationVec);
    /* Can now compute target copter rotation about its normal */
    targetRot = atan(rotationVec.y / rotationVec.x);
    targetRot = (x < 0 ? -1 * targetRot : targetRot);
    /* Use these two to find out how much we must rotate to get to target */
    undoRot = targetRot - copterRot;
    /* TODO: use PID and set both central motors to rotate */

    /* Finally, if the target Z position is higher than we want, increase the
       speed of central motors.  If less, decrease.  Probably want some sort of
       threshold here. Only do this if in auto mode. */
    if (autoMode) {
        /* TODO: use PID and set both central motors to change elevation */
    }

    return;
}

/*
 setSpeed

 Changes the copter speed without changing the direction.  To force the copter
 into a hover, one would call setSpeed(0).

 Arguments:     speed_t speed - The new speed (0 to 1).

 Returns:       None.

 Inputs:        None.

 Outputs:       None.

 Global Vars:   targetQuaternion (WRITE) - This is changed so that the copter
                    stabilizes parallel to the ground.

 Revisions:     12/04/16 - Tim Menninger: Created
                12/08/16 - Tim Menninger: Changed from setHover to setSpeed.
                    For setHover results, call setSpeed(0).
*/
void setSpeed(speed_t speed) {
    /* Set the travel speed but keep same direction */
    setMoveLateral(speed, getTargetCopterDir());

    return;
}

/*
 setDirection

 Changes the copter direction without changing the speed.

 Arguments:     angle_t degrees - Angle in degrees the copter should face,
                    relative to the initial value of 0 degrees on the copter.

 Returns:       None.

 Inputs:        None.

 Outputs:       None.

 Global Vars:   targetQuaternion (WRITE) - This is changed so that the copter
                    stabilizes parallel to the ground.

 Revisions:     12/08/16 - Tim Menninger: Created
*/
void setDirection(angle_t degrees) {
    /* Keep the speed, but change the direction. */
    setMoveLateral(getTargetCopterSpeed(), degrees);

    return;
}

/*
 setRelativeRotate

 Sets the quaternion so that the copter can begin to rotate around its yaw
 axis.  This will set the rotation relative to the current position.

 Arguments:     angle_t degrees - The number of degrees in the counterclockwise
                    direction to rotate.

 Returns:       None.

 Inputs:        None.

 Outputs:       None.

 Global Vars:   targetQuaternion (WRITE) - A rotation about the yaw axis is
                    factored in.

 Revisions:     12/06/16 - Tim Menninger: Created
*/
void setRelativeRotate(angle_t degrees) {
    /* Rotate about copter's yaw axis */
    Quaternion rotation, origTarget = getTargetQuaternion(), newTarget;
    Point3d axis = {0, 0, 1};

    /* Get the quaternion we want to factor in. */
    createQuaternion(&rotation, axis, degrees);

    /* Factor in the quaternion simply by multiplying.  This will rotate the
       copter about the yaw axis without changing its tilt. */
    multiplyQuaternions(rotation, origTarget, &newTarget);
    setTargetQuaternion(newTarget);

    return;
}

/*
 setMoveLateral

 Updates the quaternion so the copter moves at the argued speed in the argued
 direction.  The argued direction is relative to the initial forward of the
 copter, not its current direction.

 Arguments:     speed_t speed - The speed (0 to 1) of the copter, where 1 is
                    full speed and 0 will just hover.
                angle_t dir - The direction to move, relative to the initial
                    0-degree direction.

 Returns:       None.

 Inputs:        None.

 Outputs:       None.

 Global Vars:   targetQuaternion (WRITE) - The quaternion governing how the
                    copter should be oriented is overwritten

 Revisions:     12/04/16 - Tim Menninger: Created
*/
void setMoveLateral(speed_t speed, angle_t dir) {
    /* Variables used */
    Quaternion newTarget;
    Point3d axis = {1, 0, 0}, normal = {1, 0, 0};
    double mag;
    double sinT = sin(dir), cosT = cos(dir);

    /* TODO: Use the speed to determine what tilt we should use. */
    angle_t tilt = 0;

    /* Use the travel direction to determine the axis of rotation to tilt upon
       to move in that way.  We want the axis to be on the XY plane (the plane
       perpendicular to the propellers' axis).  We know tan(theta) = y/x,
       so we find tan(dir), let x = y so y = tan.  This now
       describes a vector in the XY plane in the desired direction. */
    if (cosT != 0)
        normal.y = sinT / cosT;
    /* Compensate for quadrant since we asserted x positive to start */
    if (dir > 180) {
        normal.x *= -1;
        normal.y *= -1;
    }

    /* Use tilt to compute a Z-normal that gives correct tilt.  We will then have
       a unit normal. We treat this as a right triangle, where the angle is
       the tilt, the adjacent leg is (x, y) and the opposite leg is (x, y, z).
       We use cos(tilt) = z / sqrt(x^2 + y^2) to find z. */
    normal.z = cos(tilt) * sqrt(normal.x*normal.x + normal.y*normal.y);

    /* Normalize */
    mag = sqrt(normal.x*normal.x + normal.y*normal.y + normal.z*normal.z);
    normal.x /= mag;
    normal.y /= mag;
    normal.z /= mag;

    /* We need to get quaternion that transforms (0, 0, 1)
       to that normal.  The axis this is rotated about is the cross product of
       the normal and (0, 0, 1), and the angle is still the tilt, but now it
       is the absolute value, because we always want to move toward the new
       normal. */
    axis.x = -1 * normal.y;
    axis.y = normal.x;
    axis.z = 0;

    /* Update rotation quaternion */
    createQuaternion(&newTarget, axis, abs(tilt));
    setTargetQuaternion(newTarget);

    return;
}

/*
 setMoveVertical

 Moves the copter farther or closer to the ground.  It moves up by increasing
 the speed of all propellers, and down by decreasing speeds.

 Arguments:     bool upward - True if going upward, false if downward.
                speed_t speed - Value 0 to 1 of the speed.  1 will go up at the
                    max possible speed.  0 will go down at the minimum speed
                    specified in a macro (it won't just turn propellers off
                    and crash)

 Returns:       None.

 Inputs:        None.

 Outputs:       Motors - Speed of all motors is increased or decreased

 Global Vars:   None.

 Revisions:     12/04/16 - Tim Menninger: Created empty function
*/
void setMoveVertical(bool upward, speed_t speed) {
    /* TODO: set motors */
}

/*
 initMotors

 Initializes all motors to not be on, and their rotation directions correct.

 Arguments:     None.

 Returns:       None.

 Inputs:        None.

 Outputs:       None.

 Global Vars:   hexMotors (WRITE) - PWM values set to 0 and directions set
                    appropriately, with alternating directions starting with
                    motor 0, which is set according to a macro.
                topHeliMotor (WRITE) - PWM set to 0 and direction set according
                    to a macro
                botHeliMotor (WRITE) - PWM set to 0 and direction set opposite
                    to topHeliMotor
                autoMode (WRITE) - Automode set

 Revisions:     12/04/16 - Tim Menninger: Created
*/
void initMotors() {
    /* Local variables */
    int i;
    Motor m = { 0, MOTOR_HEX0_CLKWISE };

    /* Alternate motor directions, turn all motors off */
    for (i = 0; i < 6; ++i) {
        setHexMotor(i, m);
        m.clockwise = !m.clockwise;
    }

    /* Central coaxial motors */
    m.clockwise = MOTOR_TOP_CLKWISE;
    setTopHeliMotor(m);
    m.clockwise = !m.clockwise;
    setBotHeliMotor(m);

    /* automode */
    autoMode = AUTOMODE;
}
