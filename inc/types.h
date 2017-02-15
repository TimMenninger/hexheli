/*****************************************************************************
 *
 * types.h
 *
 * Contains types and typedefs that are used across all copter files.
 *
 * Author:      Tim Menninger
 *
 *****************************************************************************/

#ifndef TYPES
#define TYPES

/* Typedefs that we use to more easily keep value sizes consistent */
typedef unsigned long   copterTime_t;
typedef long            position_t;
typedef double          angle_t;
typedef double          speed_t;
typedef double          pressure_t;
typedef double          altitude_t;
typedef double          temperature_t;

/* Define boolean datatype */
typedef enum bool {
    false = 0,
    true = 1
} bool;

#endif /* ifndef TYPES */
