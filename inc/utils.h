/*****************************************************************************
 *
 * utils.h
 *
 * Contains utilities used by the copter code.
 *
 * Author:      Tim Menninger
 *
 *****************************************************************************/

#ifndef UTILS
#define UTILS

/* Defines different statuses that can be returned by functions interracting
   with hardware */
typedef enum Status {
    SUCCESS
} Status;

/* Points in 3d */
typedef struct Point3d {
    double x;
    double y;
    double z;
} Point3d;

/* 3d vectors can also be represented as a 3d point with x, y, z */
typedef Point3d Vec3d;

#endif /* ifndef UTILS */
