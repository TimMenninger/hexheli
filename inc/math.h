/*****************************************************************************
 *
 * math.h
 *
 * Contains math functions that are devinded in C's cmath.  They are defined
 * here so we don't have to include the entire library.  Algorithms used:
 *      pi():
 *          Simply returns pi rounded to 10 digits
 *      atan():
 *          Returns arctan of the argued number
 *      sin():
 *          We preserve the sign and narrow down to 180-degree intervals.
 *          Then, because it mirrors along 90 degrees, we narrow down to
 *          <degrees> or <90 - degrees>.  Finally, because sin(x) = cos(90-x)
 *          we either compute sin(degrees) or cos(90-degrees) depending on
 *          whether the degree is less than or greater than 45, respectively.
 *          Then, we convert to radians and use the Taylor expansion to
 *          compute.
 *      cos():
 *          Exactly the same as sin without loss of generality.
 *      abs():
 *          Returns absolute value of argument.
 *      sqrt():
 *          Uses Newton method to find root of f(x) = x^2 - n where we are
 *          trying to find sqrt(n) with n >= 0.
 *      exp():
 *          Uses Taylor expansion to compute e^x.
 *
 * Author:      Tim Menninger
 *
 *****************************************************************************/

#ifndef MATH
#define MATH

#include "acosLookup.h"

/* Used to stop iteration on sqrt Newton root finder */
#define SQRT_THRESH 0.0001
/* Number of iterations to run on Taylor expansion of e^x */
#define EXP_ITERS 50

/* Approximates sine of angle */
static double sin(double degrees);
/* Approximates cosine of angle */
static double cos(double degrees);
/* Computes absolute value of argument */
static double abs(double n);
/* Approximates square root of argument */
static double sqrt(double n);
/* Returns pi rounded to 10 digits */
static double pi();
/* Returns e^x */
static double exp(double x);
/* Returns ln(x) for x near 1 (between 0.7 and 2) */
static double ln(double x);

/*
 sin

 Computes the sine of the argued angle (which is in degrees).

 Arguments:     double degrees - Degree measure of angle to compute sine of

 Returns:       double - Radian measure of sine of angle.

 Revisions:     11/25/16 - Tim Menninger: Created
*/
static double sin(double degrees) {
    /* Variables used */
    double radians;
    int sign = 1;

    /* Make argument positive and in 0-359 */
    while (degrees < 0)
        degrees += 360;
    while (degrees > 360)
        degrees -= 360;

    /* Put it in 0-179, keeping track of the sign */
    if (degrees > 180) {
        sign = -1;
        degrees -= 180;
    }

    /* If it is >90 now, we want to instead compute 90-(degrees % 90) */
    if (degrees > 90)
        degrees = 90 - (degrees - 90);

    /* Finally, if it is >45, we can just compute cos(90-x) instead */
    if (degrees > 45)
        return sign * cos(90 - degrees);

    /* Convert to radians for final computation */
    radians = degrees * pi() / 180;

    /* If here, use Taylor approximation. */
    return sign * (radians - (radians*radians*radians/6) +
        (radians*radians*radians*radians*radians/120));
}

/*
 cos

 Computes the cosine of the argued angle (which is in degrees).

 Arguments:     double degrees - Degree measure of angle to compute cosine of

 Returns:       double - Radian measure of cosine of angle.

 Revisions:     11/25/16 - Tim Menninger: Created
*/
static double cos(double degrees) {
    /* Variables used */
    double radians;
    int sign = 1;

    /* Make argument positive and in 0-359 */
    while (degrees < 0)
        degrees += 360;
    while (degrees >= 360)
        degrees -= 360;

    /* Determine the sign of the output */
    if (degrees > 90 && degrees < 270)
        sign = -1;

    /* Put it in 0-179 */
    if (degrees > 180)
        degrees -= 180;

    /* If it is >90 now, we want to instead compute 90-(degrees % 90) */
    if (degrees > 90)
        degrees = 90 - (degrees - 90);

    /* Finally, if it is >45, we can just compute cos(90-x) instead */
    if (degrees > 45)
        return sign * sin(90 - degrees);

    /* Convert to radians for final computation */
    radians = degrees * pi() / 180;

    /* If here, use Taylor approximation. */
    return sign * (1 - (radians*radians/2) +
        (radians*radians*radians*radians/24) -
        (radians*radians*radians*radians*radians*radians/720));
}

/*
 abs

 Returns absolute value of argument.

 Arguments:     double n - Value to return absolute value of

 Returns:       double - Absolute value of argument.

 Revisions:     11/25/16 - Tim Menninger: Created
*/
static double abs(double n) {
    return n > 0 ? n : -1*n;
}

/*
 sqrt

 Returns the square root of a number.

 Arguments:     double n - The number to find square root of.

 Returns:       double - The square root of n

 Revisions:     11/25/16 - Tim Menninger: Created
*/
static double sqrt(double n) {
    /* Variable used */
    double sqrtOut = 1;
    double f = 1, fp; /* f(x) and f'(x) respectively */

    /* Can't have a square root of a negative number */
    if (n < 0)
        return -1;

    /* Use Newton method to solve f(x) = x^2 - n */
     while (abs(f) > SQRT_THRESH) {
        /* Compute f and f' */
        f = sqrtOut*sqrtOut - n;
        fp = 2*sqrtOut;

        /* Compute next guess */
        sqrtOut -= (f / fp);
    }

    return sqrtOut;
}

/*
 atan

 Returns the arctan of the argument.

 Arguments:     double n - Value to find arctan of

 Returns:       double - The angle in degrees

 Revisions:     01/17/17 - Tim Menninger: Created
*/
static double atan(double n) {
    /* Keep track of the sign and then only need to compute on positive values. */
    char sign = (n >= 0 ? 1 : -1);
    bool reciprocated;
    bool identity;
    double arctan;
    double sqrtThree = sqrt(3);

    /* Use absolute value of n. */
    n = abs(n);

    /* We want to use atan(n) = pi/2 - atan(1/n) if n > 1 */
    reciprocated = (n > 1 ? true : false);
    n = (n > 1 ? 1/n : n);

    /* Reduce more using atan(n) = pi/6 + atan((n*sqrt(3)-1) / (sqrt(3)+n)) */
    identity = (n > 2 - sqrtThree ? true : false);
    n = (identity ? (n * sqrtThree - 1) / (sqrtThree + n) : n);

    /* Now compute with the reduced value. */
    arctan = n - (n*n*n)/3 + (n*n*n*n*n)/5;

    /* Now undo all of the alterations we've made and return. */
    arctan = (identity ? arctan + pi()/6 : arctan);
    arctan = (reciprocated ? pi()/2 - arctan : arctan);
    return sign * arctan * 180 / pi();
}

/*
 pi

 Returns pi rounded to 10 digits.

 Arguments:     None.

 Returns:       double - Pi rounded to 10 digits

 Revisions:     12/04/16 - Tim Menninger: Created
*/
static double pi() {
    return 3.1415926536;
}

/*
 exp

 Returns e^x by computing the Taylor series approximation.

 Arguments:     double x - Exponent in e^x.

 Returns:       double - e^x

 Revisions:     12/08/16 - Tim Menninger: Created
*/
static double exp(double x) {
    int i;
    double sum = 1;

    for (i = EXP_ITERS; i != 0; --i) {
        sum = 1 + x * sum / i;
    }

    return sum;
}

/*
 ln

 Returns ln(x) for values of x near 1 (between 0.7 and 2)

 Arguments:     double x - Value for which ln(x) is computed.

 Returns:       double - ln(x)

 Revisions:     12/08/16 - Tim Menninger: Created
*/
static double ln(double x) {
    double y = (x - 1) / (x + 1);
    double y3 = y*y*y;
    double y5 = y3*y*y;
    double y7 = y5*y*y;
    double y9 = y3*y3*y3;
    return 2 * (
        y +
        y3 / 3 +
        y5 / 5 +
        y7 / 7 +
        y9 / 9
    );
}

#endif /* ifndef MATH */
