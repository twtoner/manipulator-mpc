/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * collisionFcn_U_h_terminate.c
 *
 * Code generation for function 'collisionFcn_U_h_terminate'
 *
 */

/* Include files */
#include "collisionFcn_U_h_terminate.h"
#include "_coder_collisionFcn_U_h_mex.h"
#include "collisionFcn_U_h_data.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void collisionFcn_U_h_atexit(void)
{
  mexFunctionCreateRootTLS();
  emlrtEnterRtStackR2012b(emlrtRootTLSGlobal);
  emlrtLeaveRtStackR2012b(emlrtRootTLSGlobal);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

void collisionFcn_U_h_terminate(void)
{
  emlrtLeaveRtStackR2012b(emlrtRootTLSGlobal);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/* End of code generation (collisionFcn_U_h_terminate.c) */
