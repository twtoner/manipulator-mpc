/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * get_dn_1.c
 *
 * Code generation for function 'get_dn_1'
 *
 */

/* Include files */
#include "get_dn_1.h"
#include "collisionFcn_U_data.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"

/* Variable Definitions */
static emlrtRSInfo rd_emlrtRSI = {
    75,                                                        /* lineNo */
    "get_dn_1",                                                /* fcnName */
    "G:\\My Drive\\AEROSP 740\\project\\utilities\\get_dn_1.m" /* pathName */
};

static emlrtRSInfo sd_emlrtRSI = {
    76,                                                        /* lineNo */
    "get_dn_1",                                                /* fcnName */
    "G:\\My Drive\\AEROSP 740\\project\\utilities\\get_dn_1.m" /* pathName */
};

static emlrtRSInfo td_emlrtRSI = {
    77,                                                        /* lineNo */
    "get_dn_1",                                                /* fcnName */
    "G:\\My Drive\\AEROSP 740\\project\\utilities\\get_dn_1.m" /* pathName */
};

static emlrtRSInfo ud_emlrtRSI = {
    78,                                                        /* lineNo */
    "get_dn_1",                                                /* fcnName */
    "G:\\My Drive\\AEROSP 740\\project\\utilities\\get_dn_1.m" /* pathName */
};

static emlrtRSInfo vd_emlrtRSI = {
    79,                                                        /* lineNo */
    "get_dn_1",                                                /* fcnName */
    "G:\\My Drive\\AEROSP 740\\project\\utilities\\get_dn_1.m" /* pathName */
};

/* Function Definitions */
void get_dn_1(const emlrtStack *sp, const real_T q[7], const real_T o[21],
              real_T dn1[7])
{
  emlrtStack st;
  real_T a;
  real_T a_tmp;
  real_T b_a;
  real_T b_a_tmp;
  real_T c_a;
  real_T c_a_tmp;
  real_T d;
  real_T d1;
  real_T d2;
  real_T d_a;
  real_T d_a_tmp;
  real_T e_a;
  real_T e_a_tmp;
  real_T f_a;
  real_T mt2;
  real_T t10;
  real_T t11;
  real_T t12;
  real_T t13;
  real_T t16;
  real_T t17;
  real_T t18;
  real_T t19;
  real_T t2;
  real_T t20;
  real_T t21;
  real_T t22;
  real_T t22_tmp;
  real_T t23;
  real_T t24;
  real_T t25;
  real_T t26;
  real_T t27;
  real_T t28;
  real_T t28_tmp;
  real_T t3;
  real_T t30;
  real_T t4;
  real_T t43_tmp;
  real_T t45_tmp;
  real_T t46;
  real_T t49;
  real_T t5;
  real_T t51;
  real_T t52;
  real_T t6;
  real_T t7;
  real_T t8;
  real_T t9;
  st.prev = sp;
  st.tls = sp->tls;
  /* GET_D_1 */
  /*     DN1 = GET_D_1(O1,O2,O3,Q1,Q2,Q3,Q4,Q5,Q6) */
  /*     This function was generated by the Symbolic Math Toolbox version 8.7.
   */
  /*     08-Dec-2021 21:29:44 */
  t2 = muDoubleScalarCos(q[0]);
  t3 = muDoubleScalarCos(q[1]);
  t4 = muDoubleScalarCos(q[2]);
  t5 = muDoubleScalarCos(q[3]);
  t6 = muDoubleScalarCos(q[4]);
  t7 = muDoubleScalarCos(q[5]);
  t8 = muDoubleScalarSin(q[0]);
  t9 = muDoubleScalarSin(q[1]);
  t10 = muDoubleScalarSin(q[2]);
  t11 = muDoubleScalarSin(q[3]);
  t12 = muDoubleScalarSin(q[4]);
  t13 = muDoubleScalarSin(q[5]);
  t16 = t2 * t4;
  t17 = t2 * t10;
  t18 = t4 * t8;
  t19 = t3 * t11;
  t20 = t8 * t10;
  t21 = t3 * 0.4;
  t22_tmp = t8 * t9;
  t22 = t22_tmp * t11;
  t26 = t4 * t5 * t9;
  t28_tmp = t2 * t9;
  t28 = t28_tmp * t11;
  t30 = t28_tmp * 0.4;
  t43_tmp = t4 * t9 * t11;
  t45_tmp = t5 * t8 * t9;
  t23 = t3 * t16;
  t24 = t3 * t17;
  t25 = t3 * t18;
  t27 = t3 * t20;
  t46 = t17 + t25;
  t49 = t20 + -t23;
  t51 = t5 * t46;
  t52 = t5 * t49;
  a = muDoubleScalarAbs(o[0]);
  b_a = muDoubleScalarAbs(o[1]);
  c_a = muDoubleScalarAbs(o[2] - 0.34);
  d_a = muDoubleScalarAbs((t3 * 0.21 + -o[2]) + 0.34);
  t4 = muDoubleScalarAbs(o[0] - t28_tmp * 0.21);
  t8 = muDoubleScalarAbs(o[1] - t22_tmp * 0.21);
  a_tmp = o[1] + -(t22_tmp * 0.4);
  e_a = muDoubleScalarAbs(a_tmp);
  b_a_tmp = -o[2] + t21;
  f_a = muDoubleScalarAbs(b_a_tmp + 0.34);
  t28_tmp = muDoubleScalarAbs(o[0] - t30);
  d = (a * a + b_a * b_a) + c_a * c_a;
  st.site = &rd_emlrtRSI;
  if (d < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &st, &h_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
  }
  d = muDoubleScalarSqrt(d);
  d1 = (d_a * d_a + t4 * t4) + t8 * t8;
  st.site = &rd_emlrtRSI;
  if (d1 < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &st, &h_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
  }
  d1 = muDoubleScalarSqrt(d1);
  d2 = (e_a * e_a + f_a * f_a) + t28_tmp * t28_tmp;
  st.site = &rd_emlrtRSI;
  if (d2 < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &st, &h_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
  }
  d2 = muDoubleScalarSqrt(d2);
  c_a_tmp = t3 * t5;
  a = muDoubleScalarAbs(((b_a_tmp + c_a_tmp * 0.21) + t43_tmp * 0.21) + 0.34);
  b_a = muDoubleScalarAbs((a_tmp + t11 * (t17 * 0.21 + t25 * 0.21)) -
                          t45_tmp * 0.21);
  d_a_tmp = -o[0] + t30;
  e_a_tmp = t2 * t5 * t9;
  c_a = muDoubleScalarAbs((d_a_tmp + t11 * (t20 * 0.21 - t23 * 0.21)) +
                          e_a_tmp * 0.21);
  mt2 = (a * a + b_a * b_a) + c_a * c_a;
  st.site = &sd_emlrtRSI;
  if (mt2 < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &st, &h_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
  }
  mt2 = muDoubleScalarSqrt(mt2);
  b_a_tmp = (b_a_tmp + t5 * t21) + t43_tmp * 0.4;
  a = muDoubleScalarAbs(((b_a_tmp + t12 * (t19 * 0.0607 - t26 * 0.0607)) -
                         t6 * t9 * t10 * 0.0607) +
                        0.34);
  a_tmp = (a_tmp + t45_tmp * -0.4) + t11 * (t17 * 0.4 + t18 * t21);
  b_a = muDoubleScalarAbs((a_tmp + t6 * (t16 * 0.0607 - t27 * 0.0607)) -
                          t12 * (t22 * 0.0607 + t51 * 0.0607));
  d_a_tmp = (d_a_tmp + t5 * t30) + -t11 * (t23 * 0.4 - t20 * 0.4);
  c_a = muDoubleScalarAbs((d_a_tmp + t6 * (t18 * 0.0607 + t24 * 0.0607)) +
                          t12 * (t28 * 0.0607 - t52 * 0.0607));
  t2 = (a * a + b_a * b_a) + c_a * c_a;
  st.site = &td_emlrtRSI;
  if (t2 < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &st, &h_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
  }
  t2 = muDoubleScalarSqrt(t2);
  t3 = t11 * t49;
  t25 = t12 * (t18 + t24);
  d_a = t6 * (t28 - t52);
  a = muDoubleScalarAbs((d_a_tmp + t7 * (t3 * 0.081 + e_a_tmp * 0.081)) -
                        t13 * (t25 * 0.081 - d_a * 0.081));
  f_a = t6 * (t19 + -t26);
  e_a = t9 * t10 * t12;
  b_a =
      muDoubleScalarAbs(((b_a_tmp + t7 * (c_a_tmp * 0.081 + t43_tmp * 0.081)) +
                         t13 * (f_a * 0.081 + e_a * 0.081)) +
                        0.34);
  t22_tmp = t11 * t46;
  t28_tmp = t12 * (t16 + -t27);
  t4 = t6 * (t22 + t51);
  c_a = muDoubleScalarAbs((a_tmp + t7 * (t22_tmp * 0.081 - t45_tmp * 0.081)) -
                          t13 * (t28_tmp * 0.081 + t4 * 0.081));
  t8 = (a * a + b_a * b_a) + c_a * c_a;
  st.site = &ud_emlrtRSI;
  if (t8 < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &st, &h_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
  }
  t8 = muDoubleScalarSqrt(t8);
  a = muDoubleScalarAbs((d_a_tmp + t7 * (t3 * 0.126 + e_a_tmp * 0.126)) -
                        t13 * (t25 * 0.126 - d_a * 0.126));
  b_a =
      muDoubleScalarAbs(((b_a_tmp + t7 * (c_a_tmp * 0.126 + t43_tmp * 0.126)) +
                         t13 * (f_a * 0.126 + e_a * 0.126)) +
                        0.34);
  c_a = muDoubleScalarAbs((a_tmp + t7 * (t22_tmp * 0.126 - t45_tmp * 0.126)) -
                          t13 * (t28_tmp * 0.126 + t4 * 0.126));
  t4 = (a * a + b_a * b_a) + c_a * c_a;
  st.site = &vd_emlrtRSI;
  if (t4 < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &st, &h_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
  }
  t4 = muDoubleScalarSqrt(t4);
  dn1[0] = d;
  dn1[1] = d1;
  dn1[2] = d2;
  dn1[3] = mt2;
  dn1[4] = t2;
  dn1[5] = t8;
  dn1[6] = t4;
}

/* End of code generation (get_dn_1.c) */
