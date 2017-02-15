/*****************************************************************************
 *
 * test.c
 *
 * Unit tests for the hex heli copter code.  This tests all functions and
 * should be run if the definition of functions are changed.
 *
 * Author:      Tim Menninger
 *
 *****************************************************************************/

#include "reportTest.h"

#define SQRT_TEST_THRESH 0.01
#define TRIG_TEST_THRESH 0.0001
#define EXP_THRESH       0.0001
#define ROTATION_THRESH  0.05
#define LN_TEST_THRESH   0.0001
#define ATAN_TEST_THRESH 0.01
#define NORM_THRESH      0.01

/******************************************************************************

 test math.h

******************************************************************************/
void testSqrt() {
    /* Should return -1 on negative value */
    reportTest("sqrt() returns -1 on negative input", -1 == sqrt(-1));

    /* Should return 0 on sqrt(0) */
    reportTest("sqrt(0) returns 0", abs(sqrt(0)) <= SQRT_TEST_THRESH);

    /* Test a few values */
    reportTest("sqrt(2) close to 1.414213", (abs(1.4142135624 - sqrt(2)) <= SQRT_TEST_THRESH));
    reportTest("sqrt(0.25) close to 0.5", (abs(0.5 - sqrt(0.25)) <= SQRT_TEST_THRESH));
}

void testAbs() {
    /* abs(0) = 0 */
    reportTest("abs(0) returns 0", 0 == abs(0));
    reportTest("abs(-5) returns 5", 5 == abs(-5));
    reportTest("abs(5) returns 5", 5 == abs(5));
}

void testTrig() {
    int i, idx = 0; /* Iteration variable */
    bool works = true; /* Test report */

    /* Test sin */
    for (i = -720; i < 720 && works; ++i) {
        works &= abs(sin(i) - sinValsRadians[idx*4]) <= TRIG_TEST_THRESH;
        idx = (idx+1) % 360;
    }
    reportTest("Testing sine", works);

    /* Test cos */
    works = true;
    idx = 90; /* cos(x) = sin(90-x) */
    for (i = -720; i < 720 && works; ++i) {
        works &= abs(cos(i) - sinValsRadians[idx*4]) <= TRIG_TEST_THRESH;
        idx = (idx == 0 ? 359 : idx-1);
    }
    reportTest("Testing cosine", works);
}

void testExp() {
    /* Test 0 */
    reportTest("exp(0) returns 1", exp(0) == 1);

    /* Test negative number */
    reportTest("exp(-2.5) near 0.0821", abs(exp(-2.5) - 0.08208499) <= EXP_THRESH);

    /* Test positive number */
    reportTest("exp(5.1) near 164.0219", abs(exp(5.1) - 164.021907) <= EXP_THRESH);
}

void testLn() {
    /* Test ln(1) */
    reportTest("ln(1) returns 0", ln(1) == 0);

    /* Test close to 1 but less */
    reportTest("ln(0.75) returns near -0.2877",
        abs(ln(0.75) + 0.2876820) <= LN_TEST_THRESH
    );

    /* Test close to 1 but more */
    reportTest("ln(1.25) returns near 0.2231",
        abs(ln(1.25) - 0.2231435) <= LN_TEST_THRESH
    );
}

void testAtan() {
    /* Test values that force it through all possible flow paths. These include:
       negative numbers, numbers whose absolute value is greater than 1, and
       greater than 2 - sqrt(3) */

    /* Run tests */
    reportTest("atan(-10) returns near -84.2894",
        abs(atan(-10) + 84.2894) <= ATAN_TEST_THRESH
    );
    reportTest("atan(-1.1) returns near -47.7263",
        abs(atan(-1.1) + 47.7263) <= ATAN_TEST_THRESH
    );
    reportTest("atan(-0.9) returns near -41.9872",
        abs(atan(-0.9) + 41.9872) <= ATAN_TEST_THRESH
    );
    reportTest("atan(-0.1) returns near -5.7106",
        abs(atan(-0.1) + 5.7106) <= ATAN_TEST_THRESH
    );
    reportTest("atan(0) returns near 0",
        abs(atan(0) - 0) <= ATAN_TEST_THRESH
    );
    reportTest("atan(10) returns near 84.2894",
        abs(atan(10) - 84.2894) <= ATAN_TEST_THRESH
    );
    reportTest("atan(1.1) returns near 47.7263",
        abs(atan(1.1) - 47.7263) <= ATAN_TEST_THRESH
    );
    reportTest("atan(0.9) returns near 41.9872",
        abs(atan(0.9) - 41.9872) <= ATAN_TEST_THRESH
    );
    reportTest("atan(0.1) returns near 5.7106",
        abs(atan(0.1) - 5.7106) <= ATAN_TEST_THRESH
    );
}

void testMath() {
    printf("Testing math.h functions.\n");
    testSqrt();
    testAbs();
    testTrig();
    testExp();
    testLn();
    testAtan();
}

/******************************************************************************

 test rotation.h

******************************************************************************/

void testCreateQuaternion() {
    /* Make sure that we correctly go from axis/angle to quaternion */
    Quaternion q;
    Point3d axis = {1, -2, 3};
    angle_t angle = 63;
    Quaternion exp = {0.852640, 0.139643, -0.27928, 0.41893};

    /* Run the creation */
    createQuaternion(&q, axis, angle);
    reportTest("Creating quaternion from axis and angle",
        (abs(q.s-exp.s) <= ROTATION_THRESH) &&
        (abs(q.x-exp.x) <= ROTATION_THRESH) &&
        (abs(q.y-exp.y) <= ROTATION_THRESH) &&
        (abs(q.z-exp.z) <= ROTATION_THRESH)
    );
}

void testQuaternionToMatrix() {
    /* Variables we need */
    double x, y, z;

    /* We will test this by rotating p0 about the p axis, and see if the
       result rotation matrix applied to a few points gives what we expect. */
    RotationMatrix m;
    Quaternion q;
    Point3d p = {1, -2, 3.5}, p0 = {1, 1, -1};
    Point3d p0Exp = {-0.16900386918, 1.7034452617159, -0.264030173538};
    angle_t t = 70;
    createQuaternion(&q, p, t);
    quaternionToMatrix(q, m);

    /* Rotating point on rotation axis shouldn't change it */
    x = m[0][0] * 2 * p.x + m[0][1] * 2 * p.y + m[0][2] * 2 * p.z;
    y = m[1][0] * 2 * p.x + m[1][1] * 2 * p.y + m[1][2] * 2 * p.z;
    z = m[2][0] * 2 * p.x + m[2][1] * 2 * p.y + m[2][2] * 2 * p.z;
    reportTest("Rotating point on rotation axis doesn't change point",
        (abs(x - 2*p.x) <= ROTATION_THRESH) &&
        (abs(y - 2*p.y) <= ROTATION_THRESH) &&
        (abs(z - 2*p.z) <= ROTATION_THRESH)
    );

    /* Rotate point not on rotation axis */
    x = m[0][0] * p0.x + m[0][1] * p0.y + m[0][2] * p0.z;
    y = m[1][0] * p0.x + m[1][1] * p0.y + m[1][2] * p0.z;
    z = m[2][0] * p0.x + m[2][1] * p0.y + m[2][2] * p0.z;
    reportTest("Rotating point not on rotation axis",
        (abs(x - p0Exp.x) <= ROTATION_THRESH) &&
        (abs(y - p0Exp.y) <= ROTATION_THRESH) &&
        (abs(z - p0Exp.z) <= ROTATION_THRESH)
    );
}

void testMultiplyQuaternions() {
    /* Checks if multiplying quaternions gives correct output */
    Quaternion q1 = {0, 0, 0, 0};
    Quaternion q2 = {1, 2, -3, -2};
    Quaternion q3 = {-2, 0, 4, -2.5};
    Quaternion q2q3 = {5, 11.5, 15, 9.5};
    Quaternion out;

    /* Should return 0 */
    multiplyQuaternions(q1, q2, &out);
    reportTest("Multiplying zeroed quaternion returns 0",
        (out.s <= ROTATION_THRESH) &&
        (out.x <= ROTATION_THRESH) &&
        (out.y <= ROTATION_THRESH) &&
        (out.z <= ROTATION_THRESH)
    );

    multiplyQuaternions(q2, q3, &out);
    reportTest("Testing multiplyQuaternions()",
        (abs(out.s - q2q3.s) <= ROTATION_THRESH) &&
        (abs(out.x - q2q3.x) <= ROTATION_THRESH) &&
        (abs(out.y - q2q3.y) <= ROTATION_THRESH) &&
        (abs(out.z - q2q3.z) <= ROTATION_THRESH)
    );
}

void testQuaternion() {
    printf("Testing quaternion.h functions.\n");
    testCreateQuaternion();
    testQuaternionToMatrix();
    testMultiplyQuaternions();
}

/******************************************************************************

 test pid.c

******************************************************************************/

void testInitPid() {
    /* Test that initial values are reflected */
    PID pid;
    initPID(&pid, 1, 2, 3, 4);
    reportTest("initPID function",
        (pid.Kp == 1) &&
        (pid.Ki == 2) &&
        (pid.Kd == 3) &&
        (pid.setpoint == 4)
    );
}

void testResetPid() {
    /* Test that everything zeroes except the previous time, which should be
       the current time */
    PID pid;
    resetPID(&pid);
    reportTest("resetPID function",
        (pid.Kp == 0) &&
        (pid.Ki == 0) &&
        (pid.Kd == 0) &&
        (pid.setpoint == 0) &&
        (pid.error == 0) &&
        (pid.t0 == getCopterTime()) &&
        (pid.prevError == 0) &&
        (pid.prevT == getCopterTime()));
}

void testUpdatePid() {
    /* Test that the update function changes only the time and error */
    PID pid;
    initPID(&pid, 1, 2, 3, 4);
    pid.error = 5;
    updatePID(&pid, 6);
    reportTest("updatePID function",
        (pid.Kp == 1) &&
        (pid.Ki == 2) &&
        (pid.Kd == 3) &&
        (pid.setpoint == 4) &&
        (pid.error == 11) &&
        (pid.t0 == getCopterTime()) &&
        (pid.prevError == 6) &&
        (pid.prevT == getCopterTime())
    );
}

void testComputeGain() {
    /* Test that the compute gain function always moves us toward the setpoint,
       which isn't necessarily closer, but should be in the right +/- direction
    */
    PID pid;

    setCopterTime(1);
    reportTest("computeGain positive when currently less than setpoint",
        computeGain(&pid, 0) > 0);

    setCopterTime(2);
    reportTest("computeGain negative when currently greater than setpoint",
        computeGain(&pid, 20) <= 0);
}

void testPid() {
    printf("Testing pid.c functions.\n");
    testInitPid();
    testResetPid();
    testUpdatePid();
    testComputeGain();
}

/******************************************************************************

 test position.c

******************************************************************************/

void testPosition() {
    printf("Testing position.c functions.\n");
}

/******************************************************************************

 test copter.c

******************************************************************************/

void testCopter() {
    printf("Testing copter.c functions.\n");
}

/******************************************************************************

 Test

******************************************************************************/

int main(int argc, char **argv) {
    testMath();
    testQuaternion();
    testPid();
    testPosition();
    testCopter();

    testsOutcome();

    return 0;
}
