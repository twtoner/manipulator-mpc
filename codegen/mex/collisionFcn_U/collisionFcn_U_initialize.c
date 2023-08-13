/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * collisionFcn_U_initialize.c
 *
 * Code generation for function 'collisionFcn_U_initialize'
 *
 */

/* Include files */
#include "collisionFcn_U_initialize.h"
#include "_coder_collisionFcn_U_mex.h"
#include "collisionFcn_U_data.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void collisionFcn_U_initialize(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mex_InitInfAndNan();
  mexFunctionCreateRootTLS();
  emlrtBreakCheckR2012bFlagVar = emlrtGetBreakCheckFlagAddressR2012b();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, NULL);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/* End of code generation (collisionFcn_U_initialize.c) */
