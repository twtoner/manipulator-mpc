/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * get_dn_ddndq.h
 *
 * Code generation for function 'get_dn_ddndq'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void get_dn_ddndq(const emlrtStack *sp, const real_T q[7], const real_T O[3],
                  real_T dn[7], real_T ddndq[49], real_T h[7]);

/* End of code generation (get_dn_ddndq.h) */
