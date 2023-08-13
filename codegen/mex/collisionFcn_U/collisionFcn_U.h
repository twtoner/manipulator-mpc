/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * collisionFcn_U.h
 *
 * Code generation for function 'collisionFcn_U'
 *
 */

#pragma once

/* Include files */
#include "collisionFcn_U_types.h"
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void collisionFcn_U(const emlrtStack *sp, const emxArray_real_T *U,
                    const emxArray_real_T *O, const real_T r[7],
                    const emxArray_real_T *S, const emxArray_real_T *M,
                    const real_T q0[7], emxArray_real_T *c,
                    emxArray_real_T *gradC);

/* End of code generation (collisionFcn_U.h) */
