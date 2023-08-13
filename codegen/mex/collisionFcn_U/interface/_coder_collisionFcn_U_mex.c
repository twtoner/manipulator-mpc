/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_collisionFcn_U_mex.c
 *
 * Code generation for function '_coder_collisionFcn_U_mex'
 *
 */

/* Include files */
#include "_coder_collisionFcn_U_mex.h"
#include "_coder_collisionFcn_U_api.h"
#include "collisionFcn_U_data.h"
#include "collisionFcn_U_initialize.h"
#include "collisionFcn_U_terminate.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void collisionFcn_U_mexFunction(int32_T nlhs, mxArray *plhs[4], int32_T nrhs,
                                const mxArray *prhs[6])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *outputs[4];
  int32_T b_nlhs;
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 6) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 6, 4,
                        14, "collisionFcn_U");
  }
  if (nlhs > 4) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 14,
                        "collisionFcn_U");
  }
  /* Call the function. */
  collisionFcn_U_api(prhs, nlhs, outputs);
  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    b_nlhs = 1;
  } else {
    b_nlhs = nlhs;
  }
  emlrtReturnArrays(b_nlhs, &plhs[0], &outputs[0]);
}

void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  mexAtExit(&collisionFcn_U_atexit);
  /* Module initialization. */
  collisionFcn_U_initialize();
  /* Dispatch the entry-point. */
  collisionFcn_U_mexFunction(nlhs, plhs, nrhs, prhs);
  /* Module termination. */
  collisionFcn_U_terminate();
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2021a(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1,
                           NULL);
  return emlrtRootTLSGlobal;
}

/* End of code generation (_coder_collisionFcn_U_mex.c) */
