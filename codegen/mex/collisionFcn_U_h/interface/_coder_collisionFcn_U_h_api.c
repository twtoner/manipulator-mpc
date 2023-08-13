/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_collisionFcn_U_h_api.c
 *
 * Code generation for function '_coder_collisionFcn_U_h_api'
 *
 */

/* Include files */
#include "_coder_collisionFcn_U_h_api.h"
#include "collisionFcn_U_h.h"
#include "collisionFcn_U_h_data.h"
#include "collisionFcn_U_h_emxutil.h"
#include "collisionFcn_U_h_types.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static void b_emlrt_marshallIn(const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               emxArray_real_T *y);

static const mxArray *b_emlrt_marshallOut(void);

static void c_emlrt_marshallIn(const mxArray *O, const char_T *identifier,
                               emxArray_real_T *y);

static const mxArray *c_emlrt_marshallOut(const emxArray_real_T *u);

static void d_emlrt_marshallIn(const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               emxArray_real_T *y);

static real_T (*e_emlrt_marshallIn(const mxArray *r,
                                   const char_T *identifier))[7];

static void emlrt_marshallIn(const mxArray *U, const char_T *identifier,
                             emxArray_real_T *y);

static const mxArray *emlrt_marshallOut(const emxArray_real_T *u);

static real_T (*f_emlrt_marshallIn(const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[7];

static void g_emlrt_marshallIn(const mxArray *S, const char_T *identifier,
                               emxArray_real_T *y);

static void h_emlrt_marshallIn(const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               emxArray_real_T *y);

static void i_emlrt_marshallIn(const mxArray *M, const char_T *identifier,
                               emxArray_real_T *y);

static void j_emlrt_marshallIn(const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               emxArray_real_T *y);

static void k_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               emxArray_real_T *ret);

static void l_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               emxArray_real_T *ret);

static real_T (*m_emlrt_marshallIn(const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[7];

static void n_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               emxArray_real_T *ret);

static void o_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               emxArray_real_T *ret);

/* Function Definitions */
static void b_emlrt_marshallIn(const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               emxArray_real_T *y)
{
  k_emlrt_marshallIn(emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static const mxArray *b_emlrt_marshallOut(void)
{
  static const int32_T iv[2] = {0, 0};
  static const int32_T iv1[2] = {0, 0};
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, NULL);
  emlrtSetDimensions((mxArray *)m, &iv1[0], 2);
  emlrtAssign(&y, m);
  return y;
}

static void c_emlrt_marshallIn(const mxArray *O, const char_T *identifier,
                               emxArray_real_T *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  d_emlrt_marshallIn(emlrtAlias(O), &thisId, y);
  emlrtDestroyArray(&O);
}

static const mxArray *c_emlrt_marshallOut(const emxArray_real_T *u)
{
  static const int32_T iv[2] = {0, 0};
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, &u->data[0]);
  emlrtSetDimensions((mxArray *)m, &u->size[0], 2);
  emlrtAssign(&y, m);
  return y;
}

static void d_emlrt_marshallIn(const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               emxArray_real_T *y)
{
  l_emlrt_marshallIn(emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static real_T (*e_emlrt_marshallIn(const mxArray *r,
                                   const char_T *identifier))[7]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[7];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = f_emlrt_marshallIn(emlrtAlias(r), &thisId);
  emlrtDestroyArray(&r);
  return y;
}

static void emlrt_marshallIn(const mxArray *U, const char_T *identifier,
                             emxArray_real_T *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  b_emlrt_marshallIn(emlrtAlias(U), &thisId, y);
  emlrtDestroyArray(&U);
}

static const mxArray *emlrt_marshallOut(const emxArray_real_T *u)
{
  static const int32_T i = 0;
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericArray(1, (const void *)&i, mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, &u->data[0]);
  emlrtSetDimensions((mxArray *)m, &u->size[0], 1);
  emlrtAssign(&y, m);
  return y;
}

static real_T (*f_emlrt_marshallIn(const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[7]
{
  real_T(*y)[7];
  y = m_emlrt_marshallIn(emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static void g_emlrt_marshallIn(const mxArray *S, const char_T *identifier,
                               emxArray_real_T *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  h_emlrt_marshallIn(emlrtAlias(S), &thisId, y);
  emlrtDestroyArray(&S);
}

static void h_emlrt_marshallIn(const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               emxArray_real_T *y)
{
  n_emlrt_marshallIn(emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void i_emlrt_marshallIn(const mxArray *M, const char_T *identifier,
                               emxArray_real_T *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  j_emlrt_marshallIn(emlrtAlias(M), &thisId, y);
  emlrtDestroyArray(&M);
}

static void j_emlrt_marshallIn(const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               emxArray_real_T *y)
{
  o_emlrt_marshallIn(emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void k_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               emxArray_real_T *ret)
{
  static const int32_T dims = -1;
  int32_T i;
  int32_T i1;
  const boolean_T b = true;
  emlrtCheckVsBuiltInR2012b(emlrtRootTLSGlobal, msgId, src,
                            (const char_T *)"double", false, 1U, (void *)&dims,
                            &b, &i);
  ret->allocatedSize = i;
  i1 = ret->size[0];
  ret->size[0] = i;
  emxEnsureCapacity_real_T(ret, i1);
  ret->data = (real_T *)emlrtMxGetData(src);
  ret->canFreeData = false;
  emlrtDestroyArray(&src);
}

static void l_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               emxArray_real_T *ret)
{
  static const int32_T dims[2] = {3, -1};
  int32_T iv[2];
  int32_T i;
  const boolean_T bv[2] = {false, true};
  emlrtCheckVsBuiltInR2012b(emlrtRootTLSGlobal, msgId, src,
                            (const char_T *)"double", false, 2U,
                            (void *)&dims[0], &bv[0], &iv[0]);
  ret->allocatedSize = iv[0] * iv[1];
  i = ret->size[0] * ret->size[1];
  ret->size[0] = iv[0];
  ret->size[1] = iv[1];
  emxEnsureCapacity_real_T(ret, i);
  ret->data = (real_T *)emlrtMxGetData(src);
  ret->canFreeData = false;
  emlrtDestroyArray(&src);
}

static real_T (*m_emlrt_marshallIn(const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[7]
{
  static const int32_T dims = 7;
  real_T(*ret)[7];
  emlrtCheckBuiltInR2012b(emlrtRootTLSGlobal, msgId, src,
                          (const char_T *)"double", false, 1U, (void *)&dims);
  ret = (real_T(*)[7])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static void n_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               emxArray_real_T *ret)
{
  static const int32_T dims[2] = {-1, -1};
  int32_T iv[2];
  int32_T i;
  const boolean_T bv[2] = {true, true};
  emlrtCheckVsBuiltInR2012b(emlrtRootTLSGlobal, msgId, src,
                            (const char_T *)"double", false, 2U,
                            (void *)&dims[0], &bv[0], &iv[0]);
  ret->allocatedSize = iv[0] * iv[1];
  i = ret->size[0] * ret->size[1];
  ret->size[0] = iv[0];
  ret->size[1] = iv[1];
  emxEnsureCapacity_real_T(ret, i);
  ret->data = (real_T *)emlrtMxGetData(src);
  ret->canFreeData = false;
  emlrtDestroyArray(&src);
}

static void o_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               emxArray_real_T *ret)
{
  static const int32_T dims[2] = {-1, 7};
  int32_T iv[2];
  int32_T i;
  const boolean_T bv[2] = {true, false};
  emlrtCheckVsBuiltInR2012b(emlrtRootTLSGlobal, msgId, src,
                            (const char_T *)"double", false, 2U,
                            (void *)&dims[0], &bv[0], &iv[0]);
  ret->allocatedSize = iv[0] * iv[1];
  i = ret->size[0] * ret->size[1];
  ret->size[0] = iv[0];
  ret->size[1] = iv[1];
  emxEnsureCapacity_real_T(ret, i);
  ret->data = (real_T *)emlrtMxGetData(src);
  ret->canFreeData = false;
  emlrtDestroyArray(&src);
}

void collisionFcn_U_h_api(const mxArray *const prhs[7], int32_T nlhs,
                          const mxArray *plhs[4])
{
  emxArray_real_T *M;
  emxArray_real_T *O;
  emxArray_real_T *S;
  emxArray_real_T *U;
  emxArray_real_T *c;
  emxArray_real_T *gradC;
  emxArray_real_T *h;
  real_T(*q0)[7];
  real_T(*r)[7];
  mxMalloc(0U);
  mxMalloc(0U);
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  emxInit_real_T(&U, 1, true);
  emxInit_real_T(&O, 2, true);
  emxInit_real_T(&S, 2, true);
  emxInit_real_T(&M, 2, true);
  emxInit_real_T(&h, 2, true);
  emxInit_real_T(&c, 1, true);
  emxInit_real_T(&gradC, 2, true);
  /* Marshall function inputs */
  U->canFreeData = false;
  emlrt_marshallIn(emlrtAlias(prhs[0]), "U", U);
  O->canFreeData = false;
  c_emlrt_marshallIn(emlrtAlias(prhs[1]), "O", O);
  r = e_emlrt_marshallIn(emlrtAlias(prhs[2]), "r");
  S->canFreeData = false;
  g_emlrt_marshallIn(emlrtAlias(prhs[3]), "S", S);
  M->canFreeData = false;
  i_emlrt_marshallIn(emlrtAlias(prhs[4]), "M", M);
  q0 = e_emlrt_marshallIn(emlrtAlias(prhs[5]), "q0");
  h->canFreeData = false;
  i_emlrt_marshallIn(emlrtAlias(prhs[6]), "h", h);
  /* Invoke the target function */
  collisionFcn_U_h(U, O, *r, S, M, *q0, h, c, gradC);
  /* Marshall function outputs */
  c->canFreeData = false;
  plhs[0] = emlrt_marshallOut(c);
  emxFree_real_T(&c);
  emxFree_real_T(&h);
  emxFree_real_T(&M);
  emxFree_real_T(&S);
  emxFree_real_T(&O);
  emxFree_real_T(&U);
  if (nlhs > 1) {
    plhs[1] = b_emlrt_marshallOut();
  }
  if (nlhs > 2) {
    gradC->canFreeData = false;
    plhs[2] = c_emlrt_marshallOut(gradC);
  }
  emxFree_real_T(&gradC);
  if (nlhs > 3) {
    plhs[3] = b_emlrt_marshallOut();
  }
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

/* End of code generation (_coder_collisionFcn_U_h_api.c) */
