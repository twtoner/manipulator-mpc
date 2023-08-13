/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * collisionFcn_U.c
 *
 * Code generation for function 'collisionFcn_U'
 *
 */

/* Include files */
#include "collisionFcn_U.h"
#include "collisionFcn_U_data.h"
#include "collisionFcn_U_emxutil.h"
#include "collisionFcn_U_types.h"
#include "get_dn_ddndq.h"
#include "repmat.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include "mwmathutil.h"
#include <stddef.h>

/* Variable Definitions */
static emlrtRSInfo emlrtRSI = {
    5,                                                    /* lineNo */
    "collisionFcn_U",                                     /* fcnName */
    "G:\\My Drive\\AEROSP 740\\project\\collisionFcn_U.m" /* pathName */
};

static emlrtRSInfo b_emlrtRSI = {
    6,                                                    /* lineNo */
    "collisionFcn_U",                                     /* fcnName */
    "G:\\My Drive\\AEROSP 740\\project\\collisionFcn_U.m" /* pathName */
};

static emlrtRSInfo c_emlrtRSI = {
    7,                                                    /* lineNo */
    "collisionFcn_U",                                     /* fcnName */
    "G:\\My Drive\\AEROSP 740\\project\\collisionFcn_U.m" /* pathName */
};

static emlrtRSInfo d_emlrtRSI = {
    17,                                                   /* lineNo */
    "collisionFcn_U",                                     /* fcnName */
    "G:\\My Drive\\AEROSP 740\\project\\collisionFcn_U.m" /* pathName */
};

static emlrtRSInfo e_emlrtRSI = {
    26,                                                   /* lineNo */
    "collisionFcn_U",                                     /* fcnName */
    "G:\\My Drive\\AEROSP 740\\project\\collisionFcn_U.m" /* pathName */
};

static emlrtRSInfo f_emlrtRSI =
    {
        91,                  /* lineNo */
        "eml_mtimes_helper", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2021a\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_"
        "helper.m" /* pathName */
};

static emlrtRSInfo g_emlrtRSI =
    {
        60,                  /* lineNo */
        "eml_mtimes_helper", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2021a\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_"
        "helper.m" /* pathName */
};

static emlrtRSInfo h_emlrtRSI = {
    142,      /* lineNo */
    "mtimes", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2021a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "blas\\mtimes.m" /* pathName */
};

static emlrtRSInfo j_emlrtRSI = {
    178,           /* lineNo */
    "mtimes_blas", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2021a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "blas\\mtimes.m" /* pathName */
};

static emlrtRSInfo k_emlrtRSI = {
    29,                  /* lineNo */
    "reshapeSizeChecks", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2021a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\reshapeSizeChecks.m" /* pathName */
};

static emlrtRSInfo l_emlrtRSI = {
    109,               /* lineNo */
    "computeDimsData", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2021a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\reshapeSizeChecks.m" /* pathName */
};

static emlrtRTEInfo emlrtRTEI =
    {
        130,                   /* lineNo */
        23,                    /* colNo */
        "dynamic_size_checks", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2021a\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_"
        "helper.m" /* pName */
};

static emlrtRTEInfo b_emlrtRTEI =
    {
        135,                   /* lineNo */
        23,                    /* colNo */
        "dynamic_size_checks", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2021a\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_"
        "helper.m" /* pName */
};

static emlrtRTEInfo d_emlrtRTEI = {
    59,                  /* lineNo */
    23,                  /* colNo */
    "reshapeSizeChecks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2021a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\reshapeSizeChecks.m" /* pName */
};

static emlrtRTEInfo e_emlrtRTEI = {
    52,                  /* lineNo */
    13,                  /* colNo */
    "reshapeSizeChecks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2021a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\reshapeSizeChecks.m" /* pName */
};

static emlrtECInfo emlrtECI = {
    -1,                                                   /* nDims */
    23,                                                   /* lineNo */
    5,                                                    /* colNo */
    "collisionFcn_U",                                     /* fName */
    "G:\\My Drive\\AEROSP 740\\project\\collisionFcn_U.m" /* pName */
};

static emlrtBCInfo emlrtBCI = {
    -1,                                                    /* iFirst */
    -1,                                                    /* iLast */
    23,                                                    /* lineNo */
    23,                                                    /* colNo */
    "C",                                                   /* aName */
    "collisionFcn_U",                                      /* fName */
    "G:\\My Drive\\AEROSP 740\\project\\collisionFcn_U.m", /* pName */
    0                                                      /* checkKind */
};

static emlrtBCInfo b_emlrtBCI = {
    -1,                                                    /* iFirst */
    -1,                                                    /* iLast */
    23,                                                    /* lineNo */
    8,                                                     /* colNo */
    "C",                                                   /* aName */
    "collisionFcn_U",                                      /* fName */
    "G:\\My Drive\\AEROSP 740\\project\\collisionFcn_U.m", /* pName */
    0                                                      /* checkKind */
};

static emlrtECInfo b_emlrtECI = {
    -1,                                                   /* nDims */
    22,                                                   /* lineNo */
    5,                                                    /* colNo */
    "collisionFcn_U",                                     /* fName */
    "G:\\My Drive\\AEROSP 740\\project\\collisionFcn_U.m" /* pName */
};

static emlrtBCInfo c_emlrtBCI = {
    -1,                                                    /* iFirst */
    -1,                                                    /* iLast */
    22,                                                    /* lineNo */
    47,                                                    /* colNo */
    "dD_dQ",                                               /* aName */
    "collisionFcn_U",                                      /* fName */
    "G:\\My Drive\\AEROSP 740\\project\\collisionFcn_U.m", /* pName */
    0                                                      /* checkKind */
};

static emlrtBCInfo d_emlrtBCI = {
    -1,                                                    /* iFirst */
    -1,                                                    /* iLast */
    22,                                                    /* lineNo */
    35,                                                    /* colNo */
    "dD_dQ",                                               /* aName */
    "collisionFcn_U",                                      /* fName */
    "G:\\My Drive\\AEROSP 740\\project\\collisionFcn_U.m", /* pName */
    0                                                      /* checkKind */
};

static emlrtBCInfo e_emlrtBCI = {
    -1,                                                    /* iFirst */
    -1,                                                    /* iLast */
    22,                                                    /* lineNo */
    27,                                                    /* colNo */
    "dD_dQ",                                               /* aName */
    "collisionFcn_U",                                      /* fName */
    "G:\\My Drive\\AEROSP 740\\project\\collisionFcn_U.m", /* pName */
    0                                                      /* checkKind */
};

static emlrtBCInfo f_emlrtBCI = {
    -1,                                                    /* iFirst */
    -1,                                                    /* iLast */
    22,                                                    /* lineNo */
    12,                                                    /* colNo */
    "dD_dQ",                                               /* aName */
    "collisionFcn_U",                                      /* fName */
    "G:\\My Drive\\AEROSP 740\\project\\collisionFcn_U.m", /* pName */
    0                                                      /* checkKind */
};

static emlrtECInfo c_emlrtECI = {
    -1,                                                   /* nDims */
    19,                                                   /* lineNo */
    9,                                                    /* colNo */
    "collisionFcn_U",                                     /* fName */
    "G:\\My Drive\\AEROSP 740\\project\\collisionFcn_U.m" /* pName */
};

static emlrtBCInfo g_emlrtBCI = {
    -1,                                                    /* iFirst */
    -1,                                                    /* iLast */
    19,                                                    /* lineNo */
    27,                                                    /* colNo */
    "d_dq",                                                /* aName */
    "collisionFcn_U",                                      /* fName */
    "G:\\My Drive\\AEROSP 740\\project\\collisionFcn_U.m", /* pName */
    0                                                      /* checkKind */
};

static emlrtBCInfo h_emlrtBCI = {
    -1,                                                    /* iFirst */
    -1,                                                    /* iLast */
    19,                                                    /* lineNo */
    15,                                                    /* colNo */
    "d_dq",                                                /* aName */
    "collisionFcn_U",                                      /* fName */
    "G:\\My Drive\\AEROSP 740\\project\\collisionFcn_U.m", /* pName */
    0                                                      /* checkKind */
};

static emlrtECInfo d_emlrtECI = {
    -1,                                                   /* nDims */
    21,                                                   /* lineNo */
    10,                                                   /* colNo */
    "collisionFcn_U",                                     /* fName */
    "G:\\My Drive\\AEROSP 740\\project\\collisionFcn_U.m" /* pName */
};

static emlrtECInfo e_emlrtECI = {
    -1,                                                   /* nDims */
    18,                                                   /* lineNo */
    9,                                                    /* colNo */
    "collisionFcn_U",                                     /* fName */
    "G:\\My Drive\\AEROSP 740\\project\\collisionFcn_U.m" /* pName */
};

static emlrtBCInfo i_emlrtBCI = {
    -1,                                                    /* iFirst */
    -1,                                                    /* iLast */
    18,                                                    /* lineNo */
    25,                                                    /* colNo */
    "d_",                                                  /* aName */
    "collisionFcn_U",                                      /* fName */
    "G:\\My Drive\\AEROSP 740\\project\\collisionFcn_U.m", /* pName */
    0                                                      /* checkKind */
};

static emlrtBCInfo j_emlrtBCI = {
    -1,                                                    /* iFirst */
    -1,                                                    /* iLast */
    18,                                                    /* lineNo */
    13,                                                    /* colNo */
    "d_",                                                  /* aName */
    "collisionFcn_U",                                      /* fName */
    "G:\\My Drive\\AEROSP 740\\project\\collisionFcn_U.m", /* pName */
    0                                                      /* checkKind */
};

static emlrtBCInfo k_emlrtBCI = {
    -1,                                                    /* iFirst */
    -1,                                                    /* iLast */
    17,                                                    /* lineNo */
    51,                                                    /* colNo */
    "O",                                                   /* aName */
    "collisionFcn_U",                                      /* fName */
    "G:\\My Drive\\AEROSP 740\\project\\collisionFcn_U.m", /* pName */
    0                                                      /* checkKind */
};

static emlrtBCInfo l_emlrtBCI = {
    -1,                                                    /* iFirst */
    -1,                                                    /* iLast */
    17,                                                    /* lineNo */
    43,                                                    /* colNo */
    "q",                                                   /* aName */
    "collisionFcn_U",                                      /* fName */
    "G:\\My Drive\\AEROSP 740\\project\\collisionFcn_U.m", /* pName */
    0                                                      /* checkKind */
};

static emlrtRTEInfo f_emlrtRTEI = {
    11,                                                   /* lineNo */
    9,                                                    /* colNo */
    "collisionFcn_U",                                     /* fName */
    "G:\\My Drive\\AEROSP 740\\project\\collisionFcn_U.m" /* pName */
};

static emlrtECInfo f_emlrtECI = {
    -1,                                                   /* nDims */
    5,                                                    /* lineNo */
    5,                                                    /* colNo */
    "collisionFcn_U",                                     /* fName */
    "G:\\My Drive\\AEROSP 740\\project\\collisionFcn_U.m" /* pName */
};

static emlrtDCInfo emlrtDCI = {
    9,                                                     /* lineNo */
    15,                                                    /* colNo */
    "collisionFcn_U",                                      /* fName */
    "G:\\My Drive\\AEROSP 740\\project\\collisionFcn_U.m", /* pName */
    1                                                      /* checkKind */
};

static emlrtDCInfo b_emlrtDCI = {
    9,                                                     /* lineNo */
    23,                                                    /* colNo */
    "collisionFcn_U",                                      /* fName */
    "G:\\My Drive\\AEROSP 740\\project\\collisionFcn_U.m", /* pName */
    1                                                      /* checkKind */
};

static emlrtDCInfo c_emlrtDCI = {
    13,                                                    /* lineNo */
    18,                                                    /* colNo */
    "collisionFcn_U",                                      /* fName */
    "G:\\My Drive\\AEROSP 740\\project\\collisionFcn_U.m", /* pName */
    1                                                      /* checkKind */
};

static emlrtDCInfo d_emlrtDCI = {
    9,                                                     /* lineNo */
    1,                                                     /* colNo */
    "collisionFcn_U",                                      /* fName */
    "G:\\My Drive\\AEROSP 740\\project\\collisionFcn_U.m", /* pName */
    1                                                      /* checkKind */
};

static emlrtDCInfo e_emlrtDCI = {
    10,                                                    /* lineNo */
    1,                                                     /* colNo */
    "collisionFcn_U",                                      /* fName */
    "G:\\My Drive\\AEROSP 740\\project\\collisionFcn_U.m", /* pName */
    1                                                      /* checkKind */
};

static emlrtDCInfo f_emlrtDCI = {
    12,                                                    /* lineNo */
    5,                                                     /* colNo */
    "collisionFcn_U",                                      /* fName */
    "G:\\My Drive\\AEROSP 740\\project\\collisionFcn_U.m", /* pName */
    1                                                      /* checkKind */
};

static emlrtDCInfo g_emlrtDCI = {
    13,                                                    /* lineNo */
    5,                                                     /* colNo */
    "collisionFcn_U",                                      /* fName */
    "G:\\My Drive\\AEROSP 740\\project\\collisionFcn_U.m", /* pName */
    1                                                      /* checkKind */
};

static emlrtRTEInfo i_emlrtRTEI = {
    5,                                                    /* lineNo */
    5,                                                    /* colNo */
    "collisionFcn_U",                                     /* fName */
    "G:\\My Drive\\AEROSP 740\\project\\collisionFcn_U.m" /* pName */
};

static emlrtRTEInfo j_emlrtRTEI = {
    218,      /* lineNo */
    20,       /* colNo */
    "mtimes", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2021a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "blas\\mtimes.m" /* pName */
};

static emlrtRTEInfo k_emlrtRTEI = {
    9,                                                    /* lineNo */
    1,                                                    /* colNo */
    "collisionFcn_U",                                     /* fName */
    "G:\\My Drive\\AEROSP 740\\project\\collisionFcn_U.m" /* pName */
};

static emlrtRTEInfo l_emlrtRTEI = {
    10,                                                   /* lineNo */
    1,                                                    /* colNo */
    "collisionFcn_U",                                     /* fName */
    "G:\\My Drive\\AEROSP 740\\project\\collisionFcn_U.m" /* pName */
};

static emlrtRTEInfo m_emlrtRTEI = {
    12,                                                   /* lineNo */
    5,                                                    /* colNo */
    "collisionFcn_U",                                     /* fName */
    "G:\\My Drive\\AEROSP 740\\project\\collisionFcn_U.m" /* pName */
};

static emlrtRTEInfo n_emlrtRTEI = {
    26,                                                   /* lineNo */
    1,                                                    /* colNo */
    "collisionFcn_U",                                     /* fName */
    "G:\\My Drive\\AEROSP 740\\project\\collisionFcn_U.m" /* pName */
};

static emlrtRTEInfo o_emlrtRTEI = {
    13,                                                   /* lineNo */
    5,                                                    /* colNo */
    "collisionFcn_U",                                     /* fName */
    "G:\\My Drive\\AEROSP 740\\project\\collisionFcn_U.m" /* pName */
};

static emlrtRTEInfo p_emlrtRTEI = {
    30,                                                   /* lineNo */
    1,                                                    /* colNo */
    "collisionFcn_U",                                     /* fName */
    "G:\\My Drive\\AEROSP 740\\project\\collisionFcn_U.m" /* pName */
};

static emlrtRTEInfo q_emlrtRTEI = {
    21,                                                   /* lineNo */
    5,                                                    /* colNo */
    "collisionFcn_U",                                     /* fName */
    "G:\\My Drive\\AEROSP 740\\project\\collisionFcn_U.m" /* pName */
};

static emlrtRTEInfo r_emlrtRTEI = {
    5,                                                    /* lineNo */
    1,                                                    /* colNo */
    "collisionFcn_U",                                     /* fName */
    "G:\\My Drive\\AEROSP 740\\project\\collisionFcn_U.m" /* pName */
};

static emlrtRTEInfo s_emlrtRTEI = {
    7,                                                    /* lineNo */
    1,                                                    /* colNo */
    "collisionFcn_U",                                     /* fName */
    "G:\\My Drive\\AEROSP 740\\project\\collisionFcn_U.m" /* pName */
};

/* Function Definitions */
void collisionFcn_U(const emlrtStack *sp, const emxArray_real_T *U,
                    const emxArray_real_T *O, const real_T r[7],
                    const emxArray_real_T *S, const emxArray_real_T *M,
                    const real_T q0[7], emxArray_real_T *c,
                    emxArray_real_T *gradC)
{
  ptrdiff_t k_t;
  ptrdiff_t lda_t;
  ptrdiff_t ldb_t;
  ptrdiff_t ldc_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  emxArray_real_T *Q;
  emxArray_real_T *dD_dQ;
  emxArray_real_T *dD_dU;
  emxArray_real_T *d_;
  emxArray_real_T *d_dq;
  emxArray_real_T *r_;
  real_T ddndq[49];
  real_T a__1[7];
  real_T dn[7];
  real_T N;
  real_T alpha1;
  real_T beta1;
  real_T d;
  int32_T iv[2];
  int32_T iv1[2];
  int32_T b_loop_ub;
  int32_T c_loop_ub;
  int32_T i;
  int32_T i1;
  int32_T i2;
  int32_T i3;
  int32_T i4;
  int32_T k;
  int32_T loop_ub;
  int32_T n;
  int32_T no;
  int32_T nx;
  uint32_T u;
  char_T TRANSA1;
  char_T TRANSB1;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtCTX)sp);
  no = O->size[1];
  N = (real_T)U->size[0] / 7.0;
  st.site = &emlrtRSI;
  b_st.site = &g_emlrtRSI;
  if (U->size[0] != S->size[1]) {
    if (((S->size[0] == 1) && (S->size[1] == 1)) || (U->size[0] == 1)) {
      emlrtErrorWithMessageIdR2018a(
          &b_st, &emlrtRTEI, "Coder:toolbox:mtimes_noDynamicScalarExpansion",
          "Coder:toolbox:mtimes_noDynamicScalarExpansion", 0);
    } else {
      emlrtErrorWithMessageIdR2018a(&b_st, &b_emlrtRTEI, "MATLAB:innerdim",
                                    "MATLAB:innerdim", 0);
    }
  }
  b_st.site = &f_emlrtRSI;
  emxInit_real_T(&b_st, &Q, 1, &r_emlrtRTEI, true);
  if ((S->size[0] == 0) || (S->size[1] == 0) || (U->size[0] == 0)) {
    i = Q->size[0];
    Q->size[0] = S->size[0];
    emxEnsureCapacity_real_T(&b_st, Q, i, &i_emlrtRTEI);
    nx = S->size[0];
    for (i = 0; i < nx; i++) {
      Q->data[i] = 0.0;
    }
  } else {
    c_st.site = &h_emlrtRSI;
    d_st.site = &j_emlrtRSI;
    TRANSB1 = 'N';
    TRANSA1 = 'N';
    alpha1 = 1.0;
    beta1 = 0.0;
    m_t = (ptrdiff_t)S->size[0];
    n_t = (ptrdiff_t)1;
    k_t = (ptrdiff_t)S->size[1];
    lda_t = (ptrdiff_t)S->size[0];
    ldb_t = (ptrdiff_t)U->size[0];
    ldc_t = (ptrdiff_t)S->size[0];
    i = Q->size[0];
    Q->size[0] = S->size[0];
    emxEnsureCapacity_real_T(&d_st, Q, i, &j_emlrtRTEI);
    dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, &S->data[0], &lda_t,
          &U->data[0], &ldb_t, &beta1, &Q->data[0], &ldc_t);
  }
  st.site = &emlrtRSI;
  b_st.site = &f_emlrtRSI;
  emxInit_real_T(&b_st, &r_, 1, &s_emlrtRTEI, true);
  if (M->size[0] == 0) {
    r_->size[0] = 0;
  } else {
    c_st.site = &h_emlrtRSI;
    d_st.site = &j_emlrtRSI;
    TRANSB1 = 'N';
    TRANSA1 = 'N';
    alpha1 = 1.0;
    beta1 = 0.0;
    m_t = (ptrdiff_t)M->size[0];
    n_t = (ptrdiff_t)1;
    k_t = (ptrdiff_t)7;
    lda_t = (ptrdiff_t)M->size[0];
    ldb_t = (ptrdiff_t)7;
    ldc_t = (ptrdiff_t)M->size[0];
    i = r_->size[0];
    r_->size[0] = M->size[0];
    emxEnsureCapacity_real_T(&d_st, r_, i, &j_emlrtRTEI);
    dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, &M->data[0], &lda_t,
          &q0[0], &ldb_t, &beta1, &r_->data[0], &ldc_t);
  }
  if (Q->size[0] != r_->size[0]) {
    emlrtSizeEqCheck1DR2012b(Q->size[0], r_->size[0], &f_emlrtECI,
                             (emlrtCTX)sp);
  }
  nx = Q->size[0];
  for (i = 0; i < nx; i++) {
    Q->data[i] += r_->data[i];
  }
  st.site = &b_emlrtRSI;
  nx = Q->size[0];
  b_st.site = &k_emlrtRSI;
  c_st.site = &l_emlrtRSI;
  if (N != muDoubleScalarFloor(N)) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &c_emlrtRTEI, "Coder:MATLAB:NonIntegerInput",
        "Coder:MATLAB:NonIntegerInput", 4, 12, MIN_int32_T, 12, MAX_int32_T);
  }
  n = Q->size[0];
  if (1 > Q->size[0]) {
    n = 1;
  }
  nx = muIntScalarMax_sint32(nx, n);
  if (7 > nx) {
    emlrtErrorWithMessageIdR2018a(&st, &e_emlrtRTEI,
                                  "Coder:toolbox:reshape_emptyReshapeLimit",
                                  "Coder:toolbox:reshape_emptyReshapeLimit", 0);
  }
  if ((int32_T)N > nx) {
    emlrtErrorWithMessageIdR2018a(&st, &e_emlrtRTEI,
                                  "Coder:toolbox:reshape_emptyReshapeLimit",
                                  "Coder:toolbox:reshape_emptyReshapeLimit", 0);
  }
  if (7 * (int32_T)N != Q->size[0]) {
    emlrtErrorWithMessageIdR2018a(
        &st, &d_emlrtRTEI, "Coder:MATLAB:getReshapeDims_notSameNumel",
        "Coder:MATLAB:getReshapeDims_notSameNumel", 0);
  }
  emxInit_real_T(&st, &dD_dQ, 2, &k_emlrtRTEI, true);
  st.site = &c_emlrtRSI;
  repmat(&st, r, O->size[1], r_);
  alpha1 = 7.0 * N * (real_T)O->size[1];
  if (alpha1 != (int32_T)muDoubleScalarFloor(alpha1)) {
    emlrtIntegerCheckR2012b(alpha1, &emlrtDCI, (emlrtCTX)sp);
  }
  i = dD_dQ->size[0] * dD_dQ->size[1];
  dD_dQ->size[0] = (int32_T)alpha1;
  emxEnsureCapacity_real_T(sp, dD_dQ, i, &k_emlrtRTEI);
  alpha1 = 7.0 * N;
  if (alpha1 != muDoubleScalarFloor(alpha1)) {
    emlrtIntegerCheckR2012b(alpha1, &b_emlrtDCI, (emlrtCTX)sp);
  }
  i = dD_dQ->size[0] * dD_dQ->size[1];
  dD_dQ->size[1] = (int32_T)alpha1;
  emxEnsureCapacity_real_T(sp, dD_dQ, i, &k_emlrtRTEI);
  alpha1 = 7.0 * N * (real_T)O->size[1];
  if (alpha1 != (int32_T)muDoubleScalarFloor(alpha1)) {
    emlrtIntegerCheckR2012b(alpha1, &d_emlrtDCI, (emlrtCTX)sp);
  }
  beta1 = 7.0 * N;
  if (beta1 != muDoubleScalarFloor(beta1)) {
    emlrtIntegerCheckR2012b(beta1, &d_emlrtDCI, (emlrtCTX)sp);
  }
  nx = (int32_T)alpha1 * (int32_T)beta1;
  for (i = 0; i < nx; i++) {
    dD_dQ->data[i] = 0.0;
  }
  alpha1 = 7.0 * N * (real_T)O->size[1];
  if (alpha1 != (int32_T)muDoubleScalarFloor(alpha1)) {
    emlrtIntegerCheckR2012b(alpha1, &e_emlrtDCI, (emlrtCTX)sp);
  }
  i = c->size[0];
  c->size[0] = (int32_T)alpha1;
  emxEnsureCapacity_real_T(sp, c, i, &l_emlrtRTEI);
  alpha1 = 7.0 * N * (real_T)O->size[1];
  if (alpha1 != (int32_T)muDoubleScalarFloor(alpha1)) {
    emlrtIntegerCheckR2012b(alpha1, &e_emlrtDCI, (emlrtCTX)sp);
  }
  nx = (int32_T)alpha1;
  for (i = 0; i < nx; i++) {
    c->data[i] = 0.0;
  }
  i = (int32_T)N;
  emlrtForLoopVectorCheckR2021a(1.0, 1.0, N, mxDOUBLE_CLASS, (int32_T)N,
                                &f_emlrtRTEI, (emlrtCTX)sp);
  if (0 <= (int32_T)N - 1) {
    i1 = O->size[1];
    if (0 <= O->size[1] - 1) {
      iv[0] = 7;
      iv[1] = 7;
    }
    loop_ub = r_->size[0];
  }
  emxInit_real_T(sp, &d_, 1, &m_emlrtRTEI, true);
  emxInit_real_T(sp, &d_dq, 2, &o_emlrtRTEI, true);
  if (0 <= i - 1) {
    d = 7.0 * (real_T)no;
    b_loop_ub = (int32_T)d;
    c_loop_ub = (int32_T)d * 7;
  }
  for (k = 0; k < i; k++) {
    if (d != (int32_T)d) {
      emlrtIntegerCheckR2012b(d, &f_emlrtDCI, (emlrtCTX)sp);
    }
    i2 = d_->size[0];
    d_->size[0] = (int32_T)d;
    emxEnsureCapacity_real_T(sp, d_, i2, &m_emlrtRTEI);
    if (d != (int32_T)d) {
      emlrtIntegerCheckR2012b(d, &f_emlrtDCI, (emlrtCTX)sp);
    }
    for (i2 = 0; i2 < b_loop_ub; i2++) {
      d_->data[i2] = 0.0;
    }
    if (d != (int32_T)d) {
      emlrtIntegerCheckR2012b(d, &c_emlrtDCI, (emlrtCTX)sp);
    }
    i2 = d_dq->size[0] * d_dq->size[1];
    d_dq->size[0] = (int32_T)d;
    d_dq->size[1] = 7;
    emxEnsureCapacity_real_T(sp, d_dq, i2, &o_emlrtRTEI);
    if (d != (int32_T)d) {
      emlrtIntegerCheckR2012b(d, &g_emlrtDCI, (emlrtCTX)sp);
    }
    for (i2 = 0; i2 < c_loop_ub; i2++) {
      d_dq->data[i2] = 0.0;
    }
    for (n = 0; n < i1; n++) {
      /*          dn = d_fcn(q(:,k), O(:,i)); */
      /*          ddndq = dd_fcn(q(:,k), O(:,i)); */
      if (k + 1 > (int32_T)N) {
        emlrtDynamicBoundsCheckR2012b(k + 1, 1, (int32_T)N, &l_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      if (n + 1 > O->size[1]) {
        emlrtDynamicBoundsCheckR2012b(n + 1, 1, O->size[1], &k_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      st.site = &d_emlrtRSI;
      get_dn_ddndq(&st, *(real_T(*)[7]) & Q->data[7 * k],
                   *(real_T(*)[3]) & O->data[3 * n], dn, ddndq, a__1);
      alpha1 = (((real_T)n + 1.0) - 1.0) * 7.0 + 1.0;
      beta1 = ((real_T)n + 1.0) * 7.0;
      if (alpha1 > beta1) {
        i2 = -1;
        i3 = -1;
      } else {
        if (((int32_T)alpha1 < 1) || ((int32_T)alpha1 > d_->size[0])) {
          emlrtDynamicBoundsCheckR2012b((int32_T)alpha1, 1, d_->size[0],
                                        &j_emlrtBCI, (emlrtCTX)sp);
        }
        i2 = (int32_T)alpha1 - 2;
        if (((int32_T)beta1 < 1) || ((int32_T)beta1 > d_->size[0])) {
          emlrtDynamicBoundsCheckR2012b((int32_T)beta1, 1, d_->size[0],
                                        &i_emlrtBCI, (emlrtCTX)sp);
        }
        i3 = (int32_T)beta1 - 1;
      }
      i3 -= i2;
      if (i3 != 7) {
        emlrtSubAssignSizeCheck1dR2017a(i3, 7, &e_emlrtECI, (emlrtCTX)sp);
      }
      for (i3 = 0; i3 < 7; i3++) {
        d_->data[(i2 + i3) + 1] = dn[i3];
      }
      if (alpha1 > beta1) {
        i2 = 0;
        i3 = 0;
      } else {
        if (((int32_T)alpha1 < 1) || ((int32_T)alpha1 > d_dq->size[0])) {
          emlrtDynamicBoundsCheckR2012b((int32_T)alpha1, 1, d_dq->size[0],
                                        &h_emlrtBCI, (emlrtCTX)sp);
        }
        i2 = (int32_T)alpha1 - 1;
        if (((int32_T)beta1 < 1) || ((int32_T)beta1 > d_dq->size[0])) {
          emlrtDynamicBoundsCheckR2012b((int32_T)beta1, 1, d_dq->size[0],
                                        &g_emlrtBCI, (emlrtCTX)sp);
        }
        i3 = (int32_T)beta1;
      }
      nx = i3 - i2;
      iv1[0] = nx;
      iv1[1] = 7;
      emlrtSubAssignSizeCheckR2012b(&iv1[0], 2, &iv[0], 2, &c_emlrtECI,
                                    (emlrtCTX)sp);
      for (i3 = 0; i3 < 7; i3++) {
        for (i4 = 0; i4 < nx; i4++) {
          d_dq->data[(i2 + i4) + d_dq->size[0] * i3] = ddndq[i4 + nx * i3];
        }
      }
      if (*emlrtBreakCheckR2012bFlagVar != 0) {
        emlrtBreakCheckR2012b((emlrtCTX)sp);
      }
    }
    if (r_->size[0] != d_->size[0]) {
      emlrtSizeEqCheck1DR2012b(r_->size[0], d_->size[0], &d_emlrtECI,
                               (emlrtCTX)sp);
    }
    i2 = d_->size[0];
    d_->size[0] = r_->size[0];
    emxEnsureCapacity_real_T(sp, d_, i2, &q_emlrtRTEI);
    for (i2 = 0; i2 < loop_ub; i2++) {
      d_->data[i2] = r_->data[i2] - d_->data[i2];
    }
    alpha1 = (real_T)(k * 7) * (real_T)no + 1.0;
    u = (k + 1) * 7U;
    beta1 = (real_T)u * (real_T)no;
    if (alpha1 > beta1) {
      i2 = 0;
      i3 = 0;
    } else {
      if (((int32_T)alpha1 < 1) || ((int32_T)alpha1 > dD_dQ->size[0])) {
        emlrtDynamicBoundsCheckR2012b((int32_T)alpha1, 1, dD_dQ->size[0],
                                      &f_emlrtBCI, (emlrtCTX)sp);
      }
      i2 = (int32_T)alpha1 - 1;
      if (((int32_T)beta1 < 1) || ((int32_T)beta1 > dD_dQ->size[0])) {
        emlrtDynamicBoundsCheckR2012b((int32_T)beta1, 1, dD_dQ->size[0],
                                      &e_emlrtBCI, (emlrtCTX)sp);
      }
      i3 = (int32_T)beta1;
    }
    i4 = k * 7 + 1;
    if ((uint32_T)i4 > u) {
      i4 = 0;
      n = 0;
    } else {
      if (i4 > dD_dQ->size[1]) {
        emlrtDynamicBoundsCheckR2012b(i4, 1, dD_dQ->size[1], &d_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      i4--;
      if (((int32_T)u < 1) || ((int32_T)u > dD_dQ->size[1])) {
        emlrtDynamicBoundsCheckR2012b((int32_T)u, 1, dD_dQ->size[1],
                                      &c_emlrtBCI, (emlrtCTX)sp);
      }
      n = (int32_T)u;
    }
    iv1[0] = i3 - i2;
    iv1[1] = n - i4;
    emlrtSubAssignSizeCheckR2012b(&iv1[0], 2, &d_dq->size[0], 2, &b_emlrtECI,
                                  (emlrtCTX)sp);
    nx = d_dq->size[0];
    for (i3 = 0; i3 < 7; i3++) {
      for (n = 0; n < nx; n++) {
        dD_dQ->data[(i2 + n) + dD_dQ->size[0] * (i4 + i3)] =
            d_dq->data[n + d_dq->size[0] * i3];
      }
    }
    if (alpha1 > beta1) {
      i2 = -1;
      i3 = -1;
    } else {
      if (((int32_T)alpha1 < 1) || ((int32_T)alpha1 > c->size[0])) {
        emlrtDynamicBoundsCheckR2012b((int32_T)alpha1, 1, c->size[0],
                                      &b_emlrtBCI, (emlrtCTX)sp);
      }
      i2 = (int32_T)alpha1 - 2;
      if (((int32_T)beta1 < 1) || ((int32_T)beta1 > c->size[0])) {
        emlrtDynamicBoundsCheckR2012b((int32_T)beta1, 1, c->size[0], &emlrtBCI,
                                      (emlrtCTX)sp);
      }
      i3 = (int32_T)beta1 - 1;
    }
    nx = i3 - i2;
    if (nx != d_->size[0]) {
      emlrtSubAssignSizeCheck1dR2017a(nx, d_->size[0], &emlrtECI, (emlrtCTX)sp);
    }
    for (i3 = 0; i3 < nx; i3++) {
      c->data[(i2 + i3) + 1] = d_->data[i3];
    }
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b((emlrtCTX)sp);
    }
  }
  emxFree_real_T(&d_dq);
  emxFree_real_T(&d_);
  emxFree_real_T(&r_);
  emxFree_real_T(&Q);
  st.site = &e_emlrtRSI;
  b_st.site = &g_emlrtRSI;
  if (S->size[0] != dD_dQ->size[1]) {
    if (((dD_dQ->size[0] == 1) && (dD_dQ->size[1] == 1)) ||
        ((S->size[0] == 1) && (S->size[1] == 1))) {
      emlrtErrorWithMessageIdR2018a(
          &b_st, &emlrtRTEI, "Coder:toolbox:mtimes_noDynamicScalarExpansion",
          "Coder:toolbox:mtimes_noDynamicScalarExpansion", 0);
    } else {
      emlrtErrorWithMessageIdR2018a(&b_st, &b_emlrtRTEI, "MATLAB:innerdim",
                                    "MATLAB:innerdim", 0);
    }
  }
  b_st.site = &f_emlrtRSI;
  emxInit_real_T(&b_st, &dD_dU, 2, &n_emlrtRTEI, true);
  if ((dD_dQ->size[0] == 0) || (dD_dQ->size[1] == 0) || (S->size[0] == 0) ||
      (S->size[1] == 0)) {
    i = dD_dU->size[0] * dD_dU->size[1];
    dD_dU->size[0] = dD_dQ->size[0];
    dD_dU->size[1] = S->size[1];
    emxEnsureCapacity_real_T(&b_st, dD_dU, i, &n_emlrtRTEI);
    nx = dD_dQ->size[0] * S->size[1];
    for (i = 0; i < nx; i++) {
      dD_dU->data[i] = 0.0;
    }
  } else {
    c_st.site = &h_emlrtRSI;
    d_st.site = &j_emlrtRSI;
    TRANSB1 = 'N';
    TRANSA1 = 'N';
    alpha1 = 1.0;
    beta1 = 0.0;
    m_t = (ptrdiff_t)dD_dQ->size[0];
    n_t = (ptrdiff_t)S->size[1];
    k_t = (ptrdiff_t)dD_dQ->size[1];
    lda_t = (ptrdiff_t)dD_dQ->size[0];
    ldb_t = (ptrdiff_t)S->size[0];
    ldc_t = (ptrdiff_t)dD_dQ->size[0];
    i = dD_dU->size[0] * dD_dU->size[1];
    dD_dU->size[0] = dD_dQ->size[0];
    dD_dU->size[1] = S->size[1];
    emxEnsureCapacity_real_T(&d_st, dD_dU, i, &j_emlrtRTEI);
    dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, &dD_dQ->data[0],
          &lda_t, &S->data[0], &ldb_t, &beta1, &dD_dU->data[0], &ldc_t);
  }
  emxFree_real_T(&dD_dQ);
  i = gradC->size[0] * gradC->size[1];
  gradC->size[0] = dD_dU->size[1];
  gradC->size[1] = dD_dU->size[0];
  emxEnsureCapacity_real_T(sp, gradC, i, &p_emlrtRTEI);
  nx = dD_dU->size[0];
  for (i = 0; i < nx; i++) {
    loop_ub = dD_dU->size[1];
    for (i1 = 0; i1 < loop_ub; i1++) {
      gradC->data[i1 + gradC->size[0] * i] =
          -dD_dU->data[i + dD_dU->size[0] * i1];
    }
  }
  emxFree_real_T(&dD_dU);
  /*  Note: consider only the nearest X obstacles OR the obstalces closer than
   */
  /*  some threshold. */
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtCTX)sp);
}

/* End of code generation (collisionFcn_U.c) */
