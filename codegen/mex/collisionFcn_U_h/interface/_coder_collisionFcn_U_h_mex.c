/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_collisionFcn_U_h_mex.c
 *
 * Code generation for function '_coder_collisionFcn_U_h_mex'
 *
 */

/* Include files */
#include "_coder_collisionFcn_U_h_mex.h"
#include "_coder_collisionFcn_U_h_api.h"
#include "collisionFcn_U_h_data.h"
#include "collisionFcn_U_h_initialize.h"
#include "collisionFcn_U_h_terminate.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  mexAtExit(&collisionFcn_U_h_atexit);
  /* Module initialization. */
  collisionFcn_U_h_initialize();
  /* Dispatch the entry-point. */
  unsafe_collisionFcn_U_h_mexFunction(nlhs, plhs, nrhs, prhs);
  /* Module termination. */
  collisionFcn_U_h_terminate();
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2021a(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1,
                           NULL);
  return emlrtRootTLSGlobal;
}

void unsafe_collisionFcn_U_h_mexFunction(int32_T nlhs, mxArray *plhs[4],
                                         int32_T nrhs, const mxArray *prhs[7])
{
  const mxArray *outputs[4];
  int32_T b_nlhs;
  /* Check for proper number of arguments. */
  if (nrhs != 7) {
    emlrtErrMsgIdAndTxt(emlrtRootTLSGlobal, "EMLRT:runTime:WrongNumberOfInputs",
                        5, 12, 7, 4, 16, "collisionFcn_U_h");
  }
  if (nlhs > 4) {
    emlrtErrMsgIdAndTxt(emlrtRootTLSGlobal,
                        "EMLRT:runTime:TooManyOutputArguments", 3, 4, 16,
                        "collisionFcn_U_h");
  }
  /* Call the function. */
  collisionFcn_U_h_api(prhs, nlhs, outputs);
  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    b_nlhs = 1;
  } else {
    b_nlhs = nlhs;
  }
  emlrtReturnArrays(b_nlhs, &plhs[0], &outputs[0]);
}

/* End of code generation (_coder_collisionFcn_U_h_mex.c) */
