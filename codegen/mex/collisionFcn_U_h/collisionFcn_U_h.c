/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * collisionFcn_U_h.c
 *
 * Code generation for function 'collisionFcn_U_h'
 *
 */

/* Include files */
#include "collisionFcn_U_h.h"
#include "collisionFcn_U_h_data.h"
#include "collisionFcn_U_h_emxutil.h"
#include "collisionFcn_U_h_types.h"
#include "get_ddndq_1.h"
#include "get_ddndq_2.h"
#include "get_ddndq_3.h"
#include "get_dn_1.h"
#include "get_dn_3.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include "mwmathutil.h"
#include <stddef.h>

/* Function Definitions */
void collisionFcn_U_h(const emxArray_real_T *U, const emxArray_real_T *O,
                      const real_T r[7], const emxArray_real_T *S,
                      const emxArray_real_T *M, const real_T q0[7],
                      const emxArray_real_T *h, emxArray_real_T *c,
                      emxArray_real_T *gradC)
{
  ptrdiff_t k_t;
  ptrdiff_t lda_t;
  ptrdiff_t ldb_t;
  ptrdiff_t ldc_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  emxArray_real_T *Q;
  emxArray_real_T *dD_dQ;
  emxArray_real_T *dD_dU;
  emxArray_real_T *d_;
  emxArray_real_T *d_dq;
  emxArray_real_T *r_;
  real_T ddndq[49];
  real_T ddndq1[49];
  real_T ddndq2[49];
  real_T dn[7];
  real_T dn1[7];
  real_T dn2[7];
  real_T N;
  real_T a;
  real_T alpha1;
  real_T b_a;
  real_T b_a_tmp;
  real_T beta1;
  real_T c_a;
  real_T c_a_tmp;
  real_T d_a;
  real_T d_a_tmp;
  real_T e_a;
  real_T e_a_tmp;
  real_T f_a;
  real_T f_a_tmp;
  real_T g_a;
  real_T g_a_tmp;
  real_T h_a;
  real_T h_a_tmp;
  real_T i_a;
  real_T i_a_tmp;
  real_T j_a_tmp;
  real_T k_a_tmp;
  real_T l_a_tmp;
  real_T m_a_tmp;
  real_T n_a_tmp;
  real_T o_a_tmp;
  real_T t11;
  real_T t12;
  real_T t13;
  real_T t14;
  real_T t15;
  real_T t20;
  real_T t21;
  real_T t23;
  real_T t24;
  real_T t26;
  real_T t26_tmp;
  real_T t30_tmp;
  real_T t36_tmp;
  real_T t38_tmp;
  real_T t39;
  real_T t4;
  real_T t40;
  real_T t43_tmp;
  real_T t5;
  real_T t7;
  real_T t8;
  real_T t9;
  int32_T a_tmp;
  int32_T b_i;
  int32_T i;
  int32_T ibcol;
  int32_T itilerow;
  int32_T k;
  int32_T no;
  int32_T ntilerows;
  int32_T t8_tmp;
  int32_T t9_tmp;
  uint32_T u;
  char_T TRANSA1;
  char_T TRANSB1;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  no = O->size[1];
  N = (real_T)U->size[0] / 7.0;
  emxInit_real_T(&Q, 1, true);
  if ((S->size[0] == 0) || (S->size[1] == 0) || (U->size[0] == 0)) {
    i = Q->size[0];
    Q->size[0] = S->size[0];
    emxEnsureCapacity_real_T(Q, i);
    ibcol = S->size[0];
    for (i = 0; i < ibcol; i++) {
      Q->data[i] = 0.0;
    }
  } else {
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
    emxEnsureCapacity_real_T(Q, i);
    dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, &S->data[0], &lda_t,
          &U->data[0], &ldb_t, &beta1, &Q->data[0], &ldc_t);
  }
  emxInit_real_T(&r_, 1, true);
  if (M->size[0] == 0) {
    r_->size[0] = 0;
  } else {
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
    emxEnsureCapacity_real_T(r_, i);
    dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, &M->data[0], &lda_t,
          &q0[0], &ldb_t, &beta1, &r_->data[0], &ldc_t);
  }
  ibcol = Q->size[0];
  for (i = 0; i < ibcol; i++) {
    Q->data[i] += r_->data[i];
  }
  i = r_->size[0];
  r_->size[0] = 7 * O->size[1];
  emxEnsureCapacity_real_T(r_, i);
  ntilerows = O->size[1];
  for (itilerow = 0; itilerow < ntilerows; itilerow++) {
    ibcol = itilerow * 7;
    for (k = 0; k < 7; k++) {
      r_->data[ibcol + k] = r[k];
    }
  }
  emxInit_real_T(&dD_dQ, 2, true);
  i = dD_dQ->size[0] * dD_dQ->size[1];
  dD_dQ->size[0] = (int32_T)(7.0 * N * (real_T)O->size[1]);
  a_tmp = (int32_T)(7.0 * N);
  dD_dQ->size[1] = a_tmp;
  emxEnsureCapacity_real_T(dD_dQ, i);
  ibcol = (int32_T)(7.0 * N * (real_T)O->size[1]) * a_tmp;
  for (i = 0; i < ibcol; i++) {
    dD_dQ->data[i] = 0.0;
  }
  i = c->size[0];
  c->size[0] = (int32_T)(7.0 * N * (real_T)O->size[1]);
  emxEnsureCapacity_real_T(c, i);
  ibcol = (int32_T)(7.0 * N * (real_T)O->size[1]);
  for (i = 0; i < ibcol; i++) {
    c->data[i] = 0.0;
  }
  i = (int32_T)N;
  emxInit_real_T(&d_, 1, true);
  emxInit_real_T(&d_dq, 2, true);
  for (k = 0; k < i; k++) {
    a_tmp = d_->size[0];
    d_->size[0] = 7 * no;
    emxEnsureCapacity_real_T(d_, a_tmp);
    ibcol = 7 * no;
    for (a_tmp = 0; a_tmp < ibcol; a_tmp++) {
      d_->data[a_tmp] = 0.0;
    }
    a_tmp = d_dq->size[0] * d_dq->size[1];
    d_dq->size[0] = 7 * no;
    d_dq->size[1] = 7;
    emxEnsureCapacity_real_T(d_dq, a_tmp);
    ibcol = 7 * no * 7;
    for (a_tmp = 0; a_tmp < ibcol; a_tmp++) {
      d_dq->data[a_tmp] = 0.0;
    }
    for (b_i = 0; b_i < no; b_i++) {
      /*          dn = get_dn_3_mex(q(:,k), O(:,i)); */
      /*          ddndq = get_ddndq_3_mex(q(:,k), O(:,i)); */
      /*          dn1 = get_dn_1_mex(q(:,k), O(:,i)); */
      /*          ddndq1 = get_ddndq_1_mex(q(:,k), O(:,i)); */
      /*          dn2 = get_dn_2_mex(q(:,k), O(:,i)); */
      /*          ddndq2 = get_ddndq_2_mex(q(:,k), O(:,i)); */
      get_dn_3(*(real_T(*)[7]) & Q->data[7 * k],
               *(real_T(*)[3]) & O->data[3 * b_i], dn);
      get_ddndq_3(*(real_T(*)[7]) & Q->data[7 * k],
                  *(real_T(*)[3]) & O->data[3 * b_i], ddndq);
      get_dn_1(*(real_T(*)[7]) & Q->data[7 * k],
               *(real_T(*)[3]) & O->data[3 * b_i], dn1);
      get_ddndq_1(*(real_T(*)[7]) & Q->data[7 * k],
                  *(real_T(*)[3]) & O->data[3 * b_i], ddndq1);
      /* GET_D_2 */
      /*     DN2 = GET_D_2(O1,O2,O3,Q1,Q2,Q3,Q4,Q5,Q6) */
      /*     This function was generated by the Symbolic Math Toolbox
       * version 8.7. */
      /*     08-Dec-2021 21:29:45 */
      a = muDoubleScalarAbs(O->data[3 * b_i]);
      a_tmp = 3 * b_i + 1;
      b_a = muDoubleScalarAbs(O->data[a_tmp]);
      t4 = muDoubleScalarCos(Q->data[7 * k]);
      ntilerows = 7 * k + 1;
      t5 = muDoubleScalarCos(Q->data[ntilerows]);
      ibcol = 7 * k + 2;
      beta1 = muDoubleScalarCos(Q->data[ibcol]);
      itilerow = 7 * k + 3;
      t7 = muDoubleScalarCos(Q->data[itilerow]);
      t8_tmp = 7 * k + 4;
      t8 = muDoubleScalarCos(Q->data[t8_tmp]);
      t9_tmp = 7 * k + 5;
      t9 = muDoubleScalarCos(Q->data[t9_tmp]);
      N = muDoubleScalarSin(Q->data[7 * k]);
      t11 = muDoubleScalarSin(Q->data[ntilerows]);
      t12 = muDoubleScalarSin(Q->data[ibcol]);
      t13 = muDoubleScalarSin(Q->data[itilerow]);
      t14 = muDoubleScalarSin(Q->data[t8_tmp]);
      t15 = muDoubleScalarSin(Q->data[t9_tmp]);
      t20 = t4 * t12;
      t21 = N * t12;
      alpha1 = t5 * 0.4;
      t23 = t4 * t5 * beta1;
      t24 = t5 * beta1 * N;
      t26_tmp = t4 * t11;
      t26 = t26_tmp * 0.4;
      t36_tmp = beta1 * t11 * t13;
      t38_tmp = t7 * N * t11;
      t30_tmp = N * t11;
      t39 = t20 + t24;
      t40 = t21 + -t23;
      t43_tmp = beta1 * N;
      ntilerows = 3 * b_i + 2;
      b_a_tmp = -O->data[ntilerows] + alpha1;
      c_a_tmp = t5 * t7;
      d_a_tmp = t5 * t13;
      e_a_tmp = beta1 * t7 * t11;
      f_a_tmp = (b_a_tmp + t7 * alpha1) + t36_tmp * 0.4;
      c_a = muDoubleScalarAbs(
          ((f_a_tmp + t15 * (t8 * (d_a_tmp - e_a_tmp) * 0.081 +
                             t11 * t12 * t14 * 0.081)) +
           t9 * (c_a_tmp * 0.081 + t36_tmp * 0.081)) +
          0.34);
      g_a_tmp = O->data[a_tmp] + -(t30_tmp * 0.4);
      h_a_tmp = t7 * t39;
      i_a_tmp = t30_tmp * t13;
      j_a_tmp =
          (g_a_tmp + t38_tmp * -0.4) + t13 * (t20 * 0.4 + t43_tmp * alpha1);
      beta1 *= t4;
      k_a_tmp = t5 * t21;
      d_a = muDoubleScalarAbs(
          (j_a_tmp - t15 * (t8 * (h_a_tmp + i_a_tmp) * 0.081 +
                            t14 * (beta1 - k_a_tmp) * 0.081)) +
          t9 * (t13 * t39 * 0.081 - t38_tmp * 0.081));
      alpha1 = -O->data[3 * b_i] + t26;
      N = t4 * t7 * t11;
      l_a_tmp = t7 * t40;
      m_a_tmp = t26_tmp * t13;
      n_a_tmp = (alpha1 + t7 * t26) + -t13 * (t23 * 0.4 - t21 * 0.4);
      o_a_tmp = t5 * t20;
      e_a = muDoubleScalarAbs(
          (n_a_tmp - t15 * (t8 * (l_a_tmp - m_a_tmp) * 0.081 +
                            t14 * (t43_tmp + o_a_tmp) * 0.081)) +
          t9 * (t13 * t40 * 0.081 + N * 0.081));
      f_a = muDoubleScalarAbs(O->data[ntilerows] - 0.15);
      g_a = muDoubleScalarAbs(O->data[ntilerows] - 0.34);
      h_a = muDoubleScalarAbs((t5 * 0.21 + -O->data[ntilerows]) + 0.34);
      i_a = muDoubleScalarAbs(O->data[3 * b_i] - t26_tmp * 0.21);
      t40 = muDoubleScalarAbs(O->data[a_tmp] - t30_tmp * 0.21);
      t5 = muDoubleScalarAbs(g_a_tmp);
      t26_tmp = muDoubleScalarAbs(b_a_tmp + 0.34);
      t15 = muDoubleScalarAbs(O->data[3 * b_i] - t26);
      t9 = muDoubleScalarAbs(((b_a_tmp + c_a_tmp * 0.21) + t36_tmp * 0.21) +
                             0.34);
      t7 = muDoubleScalarAbs((g_a_tmp + t13 * (t20 * 0.21 + t24 * 0.21)) -
                             t38_tmp * 0.21);
      t4 = muDoubleScalarAbs((alpha1 + t13 * (t21 * 0.21 - t23 * 0.21)) +
                             N * 0.21);
      t39 = muDoubleScalarAbs(
          ((f_a_tmp + t14 * (d_a_tmp * 0.0607 - e_a_tmp * 0.0607)) -
           t8 * t11 * t12 * 0.0607) +
          0.34);
      beta1 = muDoubleScalarAbs(
          (j_a_tmp - t14 * (h_a_tmp * 0.0607 + i_a_tmp * 0.0607)) +
          t8 * (beta1 * 0.0607 - k_a_tmp * 0.0607));
      alpha1 = muDoubleScalarAbs(
          (n_a_tmp - t14 * (l_a_tmp * 0.0607 - m_a_tmp * 0.0607)) +
          t8 * (t43_tmp * 0.0607 + o_a_tmp * 0.0607));
      N = a * a + b_a * b_a;
      dn2[0] = muDoubleScalarSqrt(N + f_a * f_a);
      dn2[1] = muDoubleScalarSqrt(N + g_a * g_a);
      dn2[2] = muDoubleScalarSqrt((h_a * h_a + i_a * i_a) + t40 * t40);
      dn2[3] = muDoubleScalarSqrt((t5 * t5 + t26_tmp * t26_tmp) + t15 * t15);
      dn2[4] = muDoubleScalarSqrt((t9 * t9 + t7 * t7) + t4 * t4);
      dn2[5] =
          muDoubleScalarSqrt((t39 * t39 + beta1 * beta1) + alpha1 * alpha1);
      dn2[6] = muDoubleScalarSqrt((c_a * c_a + d_a * d_a) + e_a * e_a);
      get_ddndq_2(*(real_T(*)[7]) & Q->data[7 * k],
                  *(real_T(*)[3]) & O->data[3 * b_i], ddndq2);
      for (ibcol = 0; ibcol < 7; ibcol++) {
        N = h->data[b_i + h->size[0] * ibcol];
        if (N > 1.0) {
          dn[ibcol] = dn1[ibcol];
          for (a_tmp = 0; a_tmp < 7; a_tmp++) {
            ntilerows = a_tmp + 7 * ibcol;
            ddndq[ntilerows] = ddndq1[ntilerows];
          }
        } else if (N < 0.0) {
          dn[ibcol] = dn2[ibcol];
          for (a_tmp = 0; a_tmp < 7; a_tmp++) {
            ntilerows = a_tmp + 7 * ibcol;
            ddndq[ntilerows] = ddndq2[ntilerows];
          }
        }
      }
      /*          [dn, ddndq, ~] = get_dn_ddndq(q(:,k), O(:,i)); */
      N = (((real_T)b_i + 1.0) - 1.0) * 7.0 + 1.0;
      alpha1 = ((real_T)b_i + 1.0) * 7.0;
      if (N > alpha1) {
        a_tmp = -1;
        t9_tmp = 0;
      } else {
        a_tmp = (int32_T)N - 2;
        t9_tmp = (int32_T)alpha1;
      }
      ntilerows = (t9_tmp - a_tmp) - 1;
      for (t9_tmp = 0; t9_tmp < ntilerows; t9_tmp++) {
        d_->data[(a_tmp + t9_tmp) + 1] = dn[t9_tmp];
      }
      if (N > alpha1) {
        a_tmp = 0;
        t9_tmp = 0;
      } else {
        a_tmp = (int32_T)N - 1;
        t9_tmp = (int32_T)alpha1;
      }
      ntilerows = t9_tmp - a_tmp;
      for (t9_tmp = 0; t9_tmp < 7; t9_tmp++) {
        for (t8_tmp = 0; t8_tmp < ntilerows; t8_tmp++) {
          d_dq->data[(a_tmp + t8_tmp) + d_dq->size[0] * t9_tmp] =
              ddndq[t8_tmp + ntilerows * t9_tmp];
        }
      }
    }
    a_tmp = k * 7;
    N = (real_T)a_tmp * (real_T)no + 1.0;
    u = (k + 1) * 7U;
    alpha1 = (real_T)u * (real_T)no;
    if (N > alpha1) {
      t9_tmp = 1;
    } else {
      t9_tmp = (int32_T)N;
    }
    if ((uint32_T)(a_tmp + 1) > u) {
      a_tmp = 0;
    }
    ibcol = d_dq->size[0];
    for (t8_tmp = 0; t8_tmp < 7; t8_tmp++) {
      for (ntilerows = 0; ntilerows < ibcol; ntilerows++) {
        dD_dQ->data[((t9_tmp + ntilerows) + dD_dQ->size[0] * (a_tmp + t8_tmp)) -
                    1] = d_dq->data[ntilerows + d_dq->size[0] * t8_tmp];
      }
    }
    /*      dD_dQ_cell{k} = d_dq; */
    if (N > alpha1) {
      a_tmp = -1;
      t9_tmp = 0;
    } else {
      a_tmp = (int32_T)N - 2;
      t9_tmp = (int32_T)alpha1;
    }
    ibcol = (t9_tmp - a_tmp) - 1;
    for (t9_tmp = 0; t9_tmp < ibcol; t9_tmp++) {
      c->data[(a_tmp + t9_tmp) + 1] = r_->data[t9_tmp] - d_->data[t9_tmp];
    }
  }
  emxFree_real_T(&d_dq);
  emxFree_real_T(&d_);
  emxFree_real_T(&r_);
  emxFree_real_T(&Q);
  emxInit_real_T(&dD_dU, 2, true);
  if ((dD_dQ->size[0] == 0) || (dD_dQ->size[1] == 0) || (S->size[0] == 0) ||
      (S->size[1] == 0)) {
    i = dD_dU->size[0] * dD_dU->size[1];
    dD_dU->size[0] = dD_dQ->size[0];
    dD_dU->size[1] = S->size[1];
    emxEnsureCapacity_real_T(dD_dU, i);
    ibcol = dD_dQ->size[0] * S->size[1];
    for (i = 0; i < ibcol; i++) {
      dD_dU->data[i] = 0.0;
    }
  } else {
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
    emxEnsureCapacity_real_T(dD_dU, i);
    dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, &dD_dQ->data[0],
          &lda_t, &S->data[0], &ldb_t, &beta1, &dD_dU->data[0], &ldc_t);
  }
  emxFree_real_T(&dD_dQ);
  i = gradC->size[0] * gradC->size[1];
  gradC->size[0] = dD_dU->size[1];
  gradC->size[1] = dD_dU->size[0];
  emxEnsureCapacity_real_T(gradC, i);
  ibcol = dD_dU->size[0];
  for (i = 0; i < ibcol; i++) {
    ntilerows = dD_dU->size[1];
    for (a_tmp = 0; a_tmp < ntilerows; a_tmp++) {
      gradC->data[a_tmp + gradC->size[0] * i] =
          -dD_dU->data[i + dD_dU->size[0] * a_tmp];
    }
  }
  emxFree_real_T(&dD_dU);
  /*  Note: consider only the nearest X obstacles OR the obstalces closer than
   */
  /*  some threshold. */
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

/* End of code generation (collisionFcn_U_h.c) */
