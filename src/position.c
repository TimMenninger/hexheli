/*****************************************************************************
 *
 * position.c
 *
 * Contains code to read from the 9dof IMU and update global values
 * accordingly.  The 9 degrees of freedome are: 3-axis gyro, 3-axis
 * accelerometer and a 3-axis magnetometer.  It also reads from the barometer
 * and incorporates its data into the Z component of the copter position.
 *
 * Author:      Tim Menninger
 *
 *****************************************************************************/

#include "position.h"

/******************************************************************************

GLOBAL VARIABLES AND ACCESS FUNCTIONS

******************************************************************************/

/* The X- Y- and Z-coordinates of the copter, relative to some origin that is
   defined in setup.  This is to mm resolution */
Point3d       copterPosition;
void          setCopterPosition(Point3d p)      { copterPosition = p; }
Point3d       getCopterPosition()               { return copterPosition; }

/* The matrix describing the rotation one would have to take to transform
   a point from the copter's original orientation to its current one. */
Quaternion    copterQuaternion;
void          setCopterQuaternion(Quaternion q) { copterQuaternion = q; }
Quaternion    getCopterQuaternion()             { return copterQuaternion; }

/* The orientation of the copter, relative to magnetic north (0).  This is
   a value between 0 and 359. */
angle_t       copterCardinalDir;
void          setCopterCardinalDir(angle_t dir) { copterCardinalDir = dir; }
angle_t       getCopterCardinalDir()            { return copterCardinalDir; }

/* Using the accelerometer, we keep track of the last velocity and acceleration
   and the time of the last read which we can use to update the position. */
copterTime_t  prevPosT;   /* Time of last accelerometer read */
Vec3d         prevPosVelo;  /* mm/s */
Vec3d         prevPosAcc;   /* mm/(s^2) */
void          setPrevPosT(copterTime_t t)       { prevPosT    = t; }
void          setPrevPosVelo(Vec3d v)           { prevPosVelo = v; }
void          setPrevPosAcc(Vec3d a)            { prevPosAcc  = a; }
copterTime_t  getPrevPosT()                     { return prevPosT; }
Vec3d         getPrevPosVelo()                  { return prevPosVelo; }
Vec3d         getPrevPosAcc()                   { return prevPosAcc; }

/* Using the gyroscope, we can determine the orientation of the copter relative
   to the initial orientation.  Here, dimensions x = roll, y = pitch and
   z = yaw. */
copterTime_t  prevGyroT;    /* Time of last gyro read */
Vec3d         prevGyroVelo; /* degrees/s */
Vec3d         prevGyroAcc;  /* degrees/(s^2) */
void          setPrevGyroT(copterTime_t t)      { prevGyroT    = t; }
void          setPrevGyroVelo(Vec3d v)          { prevGyroVelo = v; }
void          setPrevGyroAcc(Vec3d a)           { prevGyroAcc  = a; }
copterTime_t  getPrevGyroT()                    { return prevGyroT; }
Vec3d         getPrevGyroVelo()                 { return prevGyroVelo; }
Vec3d         getPrevGyroAcc()                  { return prevGyroAcc; }

/* The roll pitch and yaw of the copter, relative to some origin that is
   defined in setup.  These are in degrees; yaw ranges from 0 to 359 while
   roll and pitch range from -179 to 180.  We will consider roll to be about X
   axis, pitch about Y axis and yaw about Z axis */
Vec3d         copterOrientation;  /* degrees */
Quaternion    imuQuaternion;
void          setCopterOrientation(Vec3d o)     { copterOrientation = o; }
void          setIMUQuaternion(Quaternion q)    { imuQuaternion     = q; }
Vec3d         getCopterOrientation()            { return copterOrientation; }
Quaternion    getIMUQuaternion()                { return imuQuaternion; }

/* Magnetometer readings. */
Vec3d         magnetometer; /* milliGauss */
void          setMagnetometer(Vec3d m)          { magnetometer = m; }
Vec3d         getMagnetometer()                 { return magnetometer; }

/* Thermometer readings */
temperature_t temperature; /* in Kelvin */
temperature_t initialTemperature; /* in Kelvin */
void          setTemperature(temperature_t t)   { temperature = t; }
void          setInitialTemp(temperature_t t)   { initialTemperature = t; }
temperature_t getTemperature()                  { return temperature; }
temperature_t getInitialTemp()                  { return initialTemperature; }

/* Barometer readings.  The pressure is the raw reading whereas the altitude
   is computed using the pressure. */
pressure_t    pressure; /* Pascals */
pressure_t    initialPressure; /* Pascals */
altitude_t    altitude; /* meters */
altitude_t    initialAltitude; /* meters */
void          setPressure(pressure_t p)         { pressure = p; }
void          setInitialPressure(pressure_t p)  { initialPressure = p; }
void          setAltitude(altitude_t a)         { altitude = a; }
void          setInitialAltitude(altitude_t a)  { initialAltitude = a; }
pressure_t    getPressure()                     { return pressure; }
pressure_t    getInitialPressure()              { return initialPressure; }
altitude_t    getAltitude()                     { return altitude; }
altitude_t    getInitialAltitude()              { return initialAltitude; }

/******************************************************************************

FUNCTION DEFINITIONS

******************************************************************************/

/*
 readAccelerometer

 Reads the accelerometer and translates it to mm/s^2.  It then puts the values
 into the global variables for acceleration in the three dimensions.  It then
 updates the previous time to the current time for reference.  Before updating
 the global accelerations, the reading undergoes the inverse rotation, bringing
 it from copter coordinates back to Earth coordinates.

 Arguments:     None.

 Returns:       Status - Describes the success or lackthereof.

 Inputs:        Accelerometer - Reads from the accelerometer on the 9DOF IMU

 Outputs:       None.

 Global Vars:   prevPosAcc (WRITE) - Writes acceleration
                prevPosT (WRITE) - Writes the time of read for reference

 Revisions:     11/14/16 - Tim Menninger: Created
*/
Status readAccelerometer() {
    /* TODO: Get actual acceleration */
    Vec3d acceleration = { 0, 0, 0 };

    /* Update acceleration */
    setPrevPosAcc(acceleration);

    /* Update the time of the last read to now */
    setPrevPosT(getCopterTime());

    return SUCCESS;
}

/*
 readGyroscope

 Reads the gyroscope and translates it to degrees/s^2.  It then puts the values
 into the global variables for gyroscopic acceleratin in the three dimensions.
 It then sets the previous time to the current time for future reference.

 Arguments:     None.

 Returns:       Status - Describes the success or lackthereof.

 Inputs:        Gyroscope - Reads from the gyroscope on the 9DOF IMU

 Outputs:       None.

 Global Vars:   prevGyroAcc (WRITE) - Writes roll, pitch and yaw acceleration
                prevGyroT (WRITE) - Writes read time for future reference

 Revisions:     11/14/16 - Tim Menninger: Created
*/
Status readGyroscope() {
    /* TODO: Get actual gyroscope reading */
    Vec3d acceleration = { 0, 0, 0 };

    /* Update gyroscopic acceleration */
    setPrevGyroAcc(acceleration);

    /* Update the time of the last gyro read to now */
    setPrevGyroT(getCopterTime());

    return SUCCESS;
}

/*
 readMagnetometer

 Reads the magnetometer and sets the magnetic field in the three dimensions.
 It converts to milliGauss before writing and returning.  The values returned
 undergo reverse rotation to transform them back to Earth coordinates from
 copter coordinates.

 Arguments:     None.

 Returns:       Status - Describes success or error

 Inputs:        Magnetometer - Reads from the magnetometer on the 9DOF IMU

 Outputs:       None.

 Global Vars:   magnetometer (WRITE) - Magnetometer readings put here

 Revisions:     11/14/16 - Tim Menninger: Created
                12/08/16 - Tim Menninger: Changed to update global value
                    instead of fill empty parameters
*/
Status readMagnetometer() {
    /* TODO: Read magnetometer */
    Vec3d magnet = { 0, 0, 0 };

    /* Set global magnet field */
    setMagnetometer(magnet);

    return SUCCESS;
}

/*
 readBarometer

 Reads the barometer and sets the global pressure value.

 Arguments:     None.

 Returns:       Status - Describes success or error

 Inputs:        barometer - Reads from the barometer sensor.

 Outputs:       None.

 Global Vars:   pressure (WRITE) - New value written.

 Revisions:     12/08/16 - Tim Menninger: Created empty function
*/
Status readBarometer() {
    /* TODO: Read barometer */
    pressure_t p = 0;

    /* Fill the global variable */
    setPressure(p);

    return SUCCESS;
}

/*
 readThermometer

 Reads the thermometer and sets the global temperature value in Kelvin.

 Arguments:     None.

 Returns:       Status - Describes success or error

 Inputs:        thermometer - Reads from the thermometer sensor.

 Outputs:       None.

 Global Vars:   pressure (WRITE) - New value written.

 Revisions:     12/08/16 - Tim Menninger: Created empty function
*/
Status readThermometer() {
    /* TODO: Read thermometer */
    temperature_t t = 273;

    /* Fill the global variable */
    setTemperature(t);

    return SUCCESS;
}

/*
 computeQuaternion

 Using the current roll, pitch and yaw, it fills the rotation matrix such that
 applying the matrix to any point would rotate it the same way that the
 copter has been rotated.

 Arguments:     None.

 Returns:       None.

 Inputs:        None.

 Outputs:       None.

 Global Vars:   rotation (WRITE) - Updated to reflect new rotation.
                copterOrientation (READ) - Read to build rotation matrix.

 Revisions:     11/14/16 - Tim Menninger: Created
*/
void computeQuaternion() {
    /* Rotation axes */
    Point3d axisX = { 1, 0, 0 };
    Point3d axisY = { 0, 1, 0 };
    Point3d axisZ = { 0, 0, 1 };

    /* Rotation matrices */
    Quaternion Qx, Qy, Qz, temp, out;

    /* Fill rotation quaternions */
    createQuaternion(&Qx, axisX, copterOrientation.x);
    createQuaternion(&Qy, axisY, copterOrientation.y);
    createQuaternion(&Qz, axisZ, copterOrientation.z);

    /* Compute overall quaternion */
    multiplyQuaternions(Qx, Qy, &temp);
    multiplyQuaternions(temp, Qz, &out);

    /* Fill quaternion */
    setIMUQuaternion(out);

    return;
}

/*
 updatePosition

 Updates the global values containing position of the copter using Euler
 integration.

 Arguments:     None.

 Returns:       Status - Describes success or error

 Inputs:        None.

 Outputs:       None.

 Global Vars:   copterPosition (WRITE) - New position written
                prevPosVelo (READ/WRITE) - Read to update position, new
                    velocity written
                prevPosAcc (READ) - Read to update position and velocity
                prevPosT (READ) - Read to compute dt

 Revisions:     11/14/16 - Tim Menninger: Created
*/
Status updatePosition() {
    /* Declare variables used */
    Status status;
    Point3d pos = getCopterPosition();
    Vec3d velocity = getPrevPosVelo();
    Vec3d accel = getPrevPosAcc();

    /* Get the time delta since the last accelerometer reading */
    copterTime_t dt = getCopterTime() - getPrevPosT();

    /* Assume that the drone has been going its current speed and acceleration */
    /* since the last reading. */
    pos.x += (velocity.x * dt) + (1/2 * accel.x * dt*dt);
    pos.y += (velocity.y * dt) + (1/2 * accel.y * dt*dt);
    pos.z += (velocity.z * dt) + (1/2 * accel.z * dt*dt);
    setCopterPosition(pos);

    /* Update the velocities based on the acceleration over the past dt */
    velocity.x += prevPosAcc.x * dt;
    velocity.y += prevPosAcc.y * dt;
    velocity.z += prevPosAcc.z * dt;
    setPrevPosVelo(velocity);

    /* Get the new accelerometer values for the next time this is called */
    status = readAccelerometer();

    return status;
}

/*
 updateRelativeOrientation

 Updates the global values containing orientation of the copter using Euler
 integration.

 Arguments:     None.

 Returns:       Status - Describes success or error

 Inputs:        None.

 Outputs:       None.

 Global Vars:   copterOrientation (WRITE) - New position written
                prevGyroVelo (READ/WRITE) - Read to update new velocity
                    written
                prevGyroAcc (READ) - Read to update and new accel written

 Revisions:     11/14/16 - Tim Menninger: Created
*/
Status updateRelativeOrientation() {
    /* Declare variables used */
    Status status;
    Vec3d orient = getCopterOrientation();
    Vec3d velocity = getPrevGyroVelo();
    Vec3d accel = getPrevGyroAcc();

    /* Get the time delta since the last gyro reading */
    copterTime_t dt = getCopterTime() - getPrevGyroT();

    /* Assume that the drone has been rotating with the same speed and */
    /* acceleration since the last reading */
    orient.x += (velocity.x * dt) + (1/2 * accel.x * dt*dt);
    orient.y += (velocity.y * dt) + (1/2 * accel.y * dt*dt);
    orient.z += (velocity.z * dt) + (1/2 * accel.z * dt*dt);
    setCopterOrientation(orient);

    /* Update the velocities based on acceleration over past dt */
    velocity.x += accel.x * dt;
    velocity.y += accel.y * dt;
    velocity.z += accel.z * dt;
    setPrevGyroVelo(velocity);

    /* Update rotation matrix based on new readings. */
    computeQuaternion();

    /* Get the new gyro reading */
    status = readGyroscope();

    return status;
}

/*
 updateAbsoluteOrientation

 Updates the global values describing absolute orientation, where 0 degrees
 is magnetic north.  It only updates in the plane parallel to the earth.

 Arguments:     None.

 Returns:       Status - Describes success or error

 Inputs:        None.

 Outputs:       None.

 Global Vars:   None.

 Revisions:     11/14/16 - Tim Menninger: Created
*/
Status updateAbsoluteOrientation() {
    /* Declare variables used */
    Status status = SUCCESS;
    angle_t dir = 0;

    /* Use the magnetometer readings to compute the absolute orientation */
    Vec3d magnet = getMagnetometer();

    /* Compute direction */
    setCopterCardinalDir(dir);

    return status;
}

/*
 updateAltitude

 Computes the altitude using the barometric pressure and temperature.  This
 can be computed by:
    h = hb + Tb/Lb * ((P/Pb)^(-R*Lb/g/M) - 1)
 where h is the altitude in meters, hb is the initial altitude in meters, Tb is
 the temperature in Kelvin at the initial altitude, Lb is the standard
 temperature lapse rate in Kelvin per meter, P is the pressure measurement, Pb
 is the pressure in the same units as P at hb, R is the universal gas constant
 in Newton-meters per mole-Kelvin, g is the gravitational constant in meters
 per second squared and M is the molar mass of air in kg per mole.

 Arguments:     None.

 Returns:       Status - Indicates success or failure.

 Inputs:        None.

 Outputs:       None.

 Global Vars:   altitude (WRITE) - New altitude written.

 Revisions:     12/08/16 - Tim Menninger: Created
*/
Status updateAltitude() {
    /* Declare variables used */
    Status status = SUCCESS;

    /* Get the values we need to compute the altitude. */
    pressure_t P = getPressure();
    pressure_t Pb = getInitialPressure();
    altitude_t hb = getInitialAltitude();
    temperature_t Tb = getInitialTemp();

    /* Constants used */
    double Lb = TEMP_LAPSE_RATE;
    double R = UNIV_GAS_K;
    double g = GRAVITY;
    double M = AIR_MOLAR_MASS;

    /* Computed altitude.  For this, instead of doing the power, we will
       use the assumption that (P/Pb) will be near 1, and the fact that
       a^b = e^(b ln a). */
    setAltitude(hb + Tb/Lb * (exp((-R*Lb/g/M) * ln(P/Pb)) - 1));

    return status;
}

/*
 initIMU

 Initializes the IMU and all of the shared variables.

 Arguments:     None.

 Returns:       None.

 Inputs:        None.

 Outputs:       None.

 Global Vars:   prevPosVelo (WRITE) - Initial value 0
                prevGyroVelo (WRITE) - Initial value 0
                copterOrientation (WRITE) - Initial value 0

 Revisions:     11/14/16 - Tim Menninger: Created
*/
void initPosition() {
    /* Initial values. */
    Quaternion q0 = { 0, 0, 0, 1 };
    Point3d p0 = { 0, 0, 0 };
    Vec3d zero = { 0, 0, 0 };

    /* Sets accelerometer readings */
    readAccelerometer();
    /* Sets gyro readings */
    readGyroscope();
    /* Sets barometer readings and altitude */
    readBarometer();
    updateAltitude();
    /* Sets magnetometer readings */
    readMagnetometer();
    /* Sets the temperature. */
    readThermometer();

    /* Set initial temperature, altitude and pressure */
    setInitialTemp(getTemperature());
    setInitialAltitude(getAltitude());
    setInitialPressure(getPressure());

    /* Set velocities and accelerations to zero */
    setPrevPosVelo(zero);
    setPrevGyroVelo(zero);
    setCopterOrientation(zero);
    setCopterQuaternion(q0);
    setCopterPosition(p0);

    /* Sets absolute position */
    updateAbsoluteOrientation();

    return;
}
