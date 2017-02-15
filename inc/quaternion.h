/*****************************************************************************
 *
 * quaternion.h
 *
 * Contains code used for using and manipulating quaternions.  Quaternions
 * represent rotations with four numbers.  Given a unit axis (ux, uy, uz)
 * and an angle t degrees to rotate around it, the quaternion (qs, qx, qy, qz)
 * is given by
 *      qs = cos(t / 2)
 *      qx = ux * sin(t / 2)
 *      qy = uy * sin(t / 2)
 *      qz = uz * sin(t / 2)
 * This will also be a unit quaternion.  We can then multiply quaternions to
 * combine rotations and generally use these to know how the copter is rotated
 * and how to rotate it to approach the target orientation.
 *
 * Author:      Tim Menninger
 *
 *****************************************************************************/

#ifndef QUATERNION
#define QUATERNION

#include "types.h"
#include "math.h"
#include "utils.h"

/* Rotation matrices can be applied to points to transform them */
typedef double RotationMatrix[3][3];

/* Quaternions represent the 3x3 rotation matrices in 4 values */
typedef struct Quaternion {
    double s;
    double x;
    double y;
    double z;
} Quaternion;

/* Multiplies two quaternions */
static void multiplyQuaternions(Quaternion, Quaternion, Quaternion*);
/* Creates a quaternion from an angle and a rotation axis vector */
static void createQuaternion(Quaternion*, Vec3d, angle_t);
/* Creates a 3x3 rotation matrix from a quaternion. */
static void quaternionToMatrix(Quaternion, RotationMatrix);
/* Inverts a quaternion */
static void invertQuaternion(Quaternion*);
/* Applies rotation quaternion onto a point */
static void applyQuaternion(Quaternion, Point3d*);

/*
 multiplyQuaternions

 Takes two quaternions and computes their product.  The product of quaternions
 q1 and q2 is
    [ s1*s2 - v1 dot v2, s1*v2 + s2*v1 + v1 cross v2 ]
 where s1 and s2 are the respective s components and v = [x, y, z] from the
 respective quaternion.

 Arguments:     Quaternion q_1 - The left quaternion in q_1*q_2 = out
                Quaternion q_2 - The right quaternion in q_1*q_2 = out
                Quaternion *out - Where to store the output

 Returns:       None.

 Revisions:     11/14/16 - Tim Menninger: Created
                11/24/16 - Tim Menninger: Changed from rotation matrices to
                    quaternions
*/
static void multiplyQuaternions(Quaternion q1, Quaternion q2, Quaternion *out) {
    /* Compute the s term */
    out->s = (q1.s*q2.s) - (q1.x*q2.x + q1.y*q2.y + q1.z*q2.z);

    /* Compute x, y and z */
    out->x = (q1.s*q2.x) + (q2.s*q1.x) + (q2.z*q1.y - q2.y*q1.z);
    out->y = (q1.s*q2.y) + (q2.s*q1.y) + (q1.z*q2.x - q1.x*q2.z);
    out->z = (q1.s*q2.z) + (q2.s*q1.z) + (q2.y*q1.x - q2.x*q1.y);

    return;
}

/*
 createQuaternion

 Creates the quaternion [q_s, q_x i + q_y j + q_z k] given the x, y, and z
 coordinates as well as the angle of rotation about the x, y, z axis.

 Arguments:     Quaternion *q - Where to store the quaternion
                Vec3d v - Represents the axis about which we rotate
                angle_t t - The degree measure angle to rotate about axis p

 Returns:       None.

 Revisions:     11/20/16 - Tim Menninger: Created
                11/24/16 - Tim Menninger: Changed from rotation matrices to
                    quaternions
*/
static void createQuaternion(Quaternion *q, Vec3d v, angle_t t) {
    /* We need our v to be normalized for the conversion. */
    float mag = sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
    /* Avoid divide by zero. */
    if (mag == 0)
        return;
    v.x /= mag;
    v.y /= mag;
    v.z /= mag;

    /* Construct the quaternion. */
    q->s = cos(t / 2);
    q->x = v.x * sin(t / 2);
    q->y = v.y * sin(t / 2);
    q->z = v.z * sin(t / 2);

    return;
}

/*
 Takes a quaternion and fills a matrix with the analogous 3x3 rotation matrix.
 A rotation matrix given a rotation quaternion, q = (qs, qx, qy, qz), is given
 by:
    1 - qy^2 - qz^2         2 * (qx*qy - qz*qs)     2 * (qx*qz + qy*qs)
    2 * (qx*qy + qz*qs)     1 - qx^2 - qz^2         2 * (qy*qz - qx*qs)
    2 * (qx*qz - qy*qs)     2 * (qy*qz + qx*qs)     1 - qx^2 - qy^2

 Arguments:     Quaternion q - The quaternion that we are creating a rotation
                    matrix for.
                RotationMatrix m - The rotation matrix to fill

 Returns:       None.

 Revisions:     11/24/16 - Tim Menninger: Created
*/
static void quaternionToMatrix(Quaternion q, RotationMatrix m) {
    /* Ensure pointer validity */
    if (!m)
        return;

    /* Fill m with rotation matrix that q represents. */
    m[0][0] = 1 - (2*q.y*q.y) - (2*q.z*q.z);
    m[0][1] = 2 * (q.x*q.y - q.z*q.s);
    m[0][2] = 2 * (q.x*q.z + q.y*q.s);

    m[1][0] = 2 * (q.x*q.y + q.z*q.s);
    m[1][1] = 1 - (2*q.x*q.x) - (2*q.z*q.z);
    m[1][2] = 2 * (q.y*q.z - q.x*q.s);

    m[2][0] = 2 * (q.x*q.z - q.y*q.s);
    m[2][1] = 2 * (q.y*q.z + q.x*q.s);
    m[2][2] = 1 - (2*q.x*q.x) - (2*q.y*q.y);
}

/*
 invertQuaternion

 Returns the inverse of the quaternion.  This assumes that the quaternion
 argued is a unit quaternion.

 Arguments:     Quaternion q - The quaternion to find the inverse of.

 Returns:       None.

 Revisions:     01/11/17 - Tim Menninger: Created
*/
static void invertQuaternion(Quaternion *q) {
    /* The inverse of quaternion q = q + qi + qj + qk is
       q_inv = (q - qi - qj - qk) / ||q||^2
       However, we know that ||q|| = 1 */
    q->s = q->s;
    q->x = -1 * q->x;
    q->y = -1 * q->y;
    q->z = -1 * q->z;

    return;
}

/*
 applyQuaternion

 Takes a quaternion and applies the rotation it represents onto a point.

 Arguments:     Quaternion q - The quaternion to apply to a vector
                Point3d *p - The point the rotation is applied to

 Returns:       None.

 Revisions:     01/17/17 - Tim Menninger: Created
*/
static void applyQuaternion(Quaternion q, Point3d *p) {
    /* Copy the argued point as reference when rotating it. */
    Point3d orig = *p;

    /* Need a 3x3 rotation matrix from the quaternion */
    RotationMatrix m;
    quaternionToMatrix(q, m);

    /* Apply the rotation to the point p, in place */
    p->x = m[0][0]*orig.x + m[0][1]*orig.y + m[0][2]*orig.z;
    p->y = m[1][0]*orig.x + m[1][1]*orig.y + m[1][2]*orig.z;
    p->z = m[2][0]*orig.x + m[2][1]*orig.y + m[2][2]*orig.z;
}

#endif /* ifndef QUATERNION */
