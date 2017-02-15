#ifndef REPORTTEST
#define REPORTTEST

/* Copter functions */
#include <copter.h>
#include <pid.h>
#include <position.h>
#include <quaternion.h>

/* Test helpers */
#include "sinTable.h"

/* C libraries */
#include <stdio.h>
#include <string.h>

/* Test printing macros */
#define TEST_OUT_LEN 70
#define KNRM  "\x1B[0m"  /* Normal color */
#define KRED  "\x1B[31m" /* Color for faling test */
#define KGRN  "\x1B[32m" /* Color for passing test */
#define KCYN  "\x1B[36m" /* Color for test headers */

/* Keep track of how many tests passed. */
static int numTests  = 0;
static int numFailed = 0;

static void reportTest(char *testName, bool passed) {
    /* Variable used in iteration */
    int i;

    /* Assert pointer is valid */
    if (!testName)
        return;

    /* Count this test. */
    numTests++;
    if (!passed)
        numFailed++;

    /* Print the test name, whether it was passed, and a newline. */
    printf("    %s%s", KCYN, testName);
    /* Pad with spaces */
    for (i = 0; i < TEST_OUT_LEN - 4 - 4; ++i) {
        if (*testName != 0)
            testName++;
        else
            printf(".");
    }
    if (passed)
        printf("%sPASS%s", KGRN, KNRM);
    else
        printf("%sFAIL%s", KRED, KNRM);
    printf("\n");

    return;
}

/* Report all tests status */
static void testsOutcome() {
    if (numFailed == 0)
        printf("\nAll %d tests passed!\n\n", numTests);
    else
        printf("\n%d of %d tests failed!\n\n", numFailed, numTests);
}

#endif /* ifndef REPORTTEST */
