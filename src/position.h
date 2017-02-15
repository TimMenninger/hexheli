/*****************************************************************************
 *
 * position.h
 *
 * Contains function handles and macros for position.c
 *
 * Author:      Tim Menninger
 *
 *****************************************************************************/

#ifndef POSITION
#define POSITION

#include "common.h"

/* Constants used for computing altitude */
#define TEMP_LAPSE_RATE -0.0065 /* K/m */
#define UNIV_GAS_K 8.31432 /* N*m/(mol*K) */
#define GRAVITY 9.80665 /* m/s^2 */
#define AIR_MOLAR_MASS 0.0289644 /* kg/mol */

/* Accessors and mutators for shared variables */
void setCopterPosition(Point3d);
Point3d getCopterPosition();
void setCopterQuaternion(Quaternion);
Quaternion getCopterQuaternion();
void setCopterCardinalDir(angle_t);
angle_t getCopterCardinalDir();
void setPrevPosT(copterTime_t t);
void setPrevPosVelo(Vec3d v);
void setPrevPosAcc(Vec3d a);
copterTime_t getPrevPosT();
Vec3d getPrevPosVelo();
Vec3d getPrevPosAcc();
void setPrevGyroT(copterTime_t t);
void setPrevGyroVelo(Vec3d v);
void setPrevGyroAcc(Vec3d a);
copterTime_t getPrevGyroT();
Vec3d getPrevGyroVelo();
Vec3d getPrevGyroAcc();
void setCopterOrientation(Vec3d o);
void setIMUQuaternion(Quaternion q);
Vec3d getCopterOrientation();
Quaternion getIMUQuaternion();
void setMagnetometer(Vec3d m);
Vec3d getMagnetometer();
void setTemperature(temperature_t t);
void setInitialTemp(temperature_t t);
temperature_t getTemperature();
temperature_t getInitialTemp();
void setPressure(pressure_t p);
void setInitialPressure(pressure_t p);
void setAltitude(altitude_t a);
void setInitialAltitude(altitude_t a);
pressure_t getPressure();
pressure_t getInitialPressure();
altitude_t getAltitude();
altitude_t getInitialAltitude();

/* Reads accelerometer */
Status readAccelerometer();
/* Reads gyroscope */
Status readGyroscope();
/* Reads magnetometer */
Status readMagnetometer();
/* Reads barometer */
Status readBarometer();
/* Reads thermometer */
Status readThermometer();
/* Computes rotation matrix from roll pitch and yaw */
void computeQuaternion();
/* Updates known position variables from accelerometer */
Status updatePosition();
/* Updates altitude based on pressure reading */
Status updateAltitude();
/* Updates known relative orientation from gyro */
Status updateRelativeOrientation();
/* Updates known absolute orientation (NESW) from magnetometer */
Status updateAbsoluteOrientation();
/* Initializes shared variables */
void initPosition();

#endif /* ifndef POSITION */
