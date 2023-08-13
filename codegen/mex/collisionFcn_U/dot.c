/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * dot.c
 *
 * Code generation for function 'dot'
 *
 */

/* Include files */
#include "dot.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void dot(const real_T a[21], const real_T b[21], real_T c[7])
{
  int32_T i;
  int32_T i1;
  int32_T i2;
  i2 = 1;
  for (i = 0; i < 7; i++) {
    i1 = i2 - 1;
    i2 += 3;
    c[i] = (a[i1] * b[i1] + a[i1 + 1] * b[i1 + 1]) + a[i1 + 2] * b[i1 + 2];
  }
}

/* End of code generation (dot.c) */
