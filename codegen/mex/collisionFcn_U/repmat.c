/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * repmat.c
 *
 * Code generation for function 'repmat'
 *
 */

/* Include files */
#include "repmat.h"
#include "collisionFcn_U_data.h"
#include "collisionFcn_U_emxutil.h"
#include "collisionFcn_U_types.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"

/* Variable Definitions */
static emlrtRSInfo i_emlrtRSI = {
    21,                               /* lineNo */
    "eml_int_forloop_overflow_check", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2021a\\toolbox\\eml\\lib\\matlab\\eml\\eml_int_forloop_"
    "overflow_check.m" /* pathName */
};

static emlrtRSInfo m_emlrtRSI = {
    28,       /* lineNo */
    "repmat", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2021a\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m" /* pathName
                                                                         */
};

static emlrtRSInfo n_emlrtRSI = {
    69,       /* lineNo */
    "repmat", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2021a\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m" /* pathName
                                                                         */
};

static emlrtMCInfo emlrtMCI = {
    47,       /* lineNo */
    5,        /* colNo */
    "repmat", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2021a\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m" /* pName
                                                                         */
};

static emlrtRTEInfo t_emlrtRTEI = {
    59,       /* lineNo */
    28,       /* colNo */
    "repmat", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2021a\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m" /* pName
                                                                         */
};

static emlrtRSInfo yf_emlrtRSI = {
    47,       /* lineNo */
    "repmat", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2021a\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m" /* pathName
                                                                         */
};

/* Function Declarations */
static void error(const emlrtStack *sp, const mxArray *b,
                  emlrtMCInfo *location);

/* Function Definitions */
static void error(const emlrtStack *sp, const mxArray *b, emlrtMCInfo *location)
{
  const mxArray *pArray;
  pArray = b;
  emlrtCallMATLABR2012b((emlrtCTX)sp, 0, NULL, 1, &pArray,
                        (const char_T *)"error", true, location);
}

void repmat(const emlrtStack *sp, const real_T a[7], real_T varargin_1,
            emxArray_real_T *b)
{
  static const int32_T iv[2] = {1, 15};
  static const char_T u[15] = {'M', 'A', 'T', 'L', 'A', 'B', ':', 'p',
                               'm', 'a', 'x', 's', 'i', 'z', 'e'};
  emlrtStack b_st;
  emlrtStack st;
  const mxArray *m;
  const mxArray *y;
  int32_T ibcol;
  int32_T itilerow;
  int32_T k;
  int32_T outsize_idx_0;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &m_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  if (varargin_1 != muDoubleScalarFloor(varargin_1)) {
    emlrtErrorWithMessageIdR2018a(
        &st, &c_emlrtRTEI, "Coder:MATLAB:NonIntegerInput",
        "Coder:MATLAB:NonIntegerInput", 4, 12, MIN_int32_T, 12, MAX_int32_T);
  }
  outsize_idx_0 = 7 * (int32_T)varargin_1;
  if (!(outsize_idx_0 == 7.0 * (real_T)(int32_T)varargin_1)) {
    y = NULL;
    m = emlrtCreateCharArray(2, &iv[0]);
    emlrtInitCharArrayR2013a((emlrtCTX)sp, 15, m, &u[0]);
    emlrtAssign(&y, m);
    st.site = &yf_emlrtRSI;
    error(&st, y, &emlrtMCI);
  }
  ibcol = b->size[0];
  b->size[0] = outsize_idx_0;
  emxEnsureCapacity_real_T(sp, b, ibcol, &t_emlrtRTEI);
  outsize_idx_0 = (int32_T)varargin_1;
  st.site = &n_emlrtRSI;
  if ((1 <= (int32_T)varargin_1) && ((int32_T)varargin_1 > 2147483646)) {
    b_st.site = &i_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (itilerow = 0; itilerow < outsize_idx_0; itilerow++) {
    ibcol = itilerow * 7;
    for (k = 0; k < 7; k++) {
      b->data[ibcol + k] = a[k];
    }
  }
}

/* End of code generation (repmat.c) */
