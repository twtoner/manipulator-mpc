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
#include "rt_nonfinite.h"
#include "mwmathutil.h"

/* Function Definitions */
void get_dn_1(const real_T q[7], const real_T o[3], real_T dn1[7])
{
  real_T a;
  real_T a_tmp;
  real_T b_a;
  real_T b_a_tmp;
  real_T c_a;
  real_T c_a_tmp;
  real_T d_a;
  real_T d_a_tmp;
  real_T e_a;
  real_T e_a_tmp;
  real_T f_a;
  real_T g_a;
  real_T h_a;
  real_T i_a;
  real_T j_a;
  real_T k_a;
  real_T l_a;
  real_T m_a;
  real_T n_a;
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
  real_T t24;
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
  t8 = t3 * t16;
  t24 = t3 * t17;
  t4 = t3 * t18;
  t27 = t3 * t20;
  t46 = t17 + t4;
  t49 = t20 + -t8;
  t51 = t5 * t46;
  t52 = t5 * t49;
  a = muDoubleScalarAbs(o[0]);
  b_a = muDoubleScalarAbs(o[1]);
  c_a = muDoubleScalarAbs(o[2] - 0.34);
  d_a = muDoubleScalarAbs((t3 * 0.21 + -o[2]) + 0.34);
  e_a = muDoubleScalarAbs(o[0] - t28_tmp * 0.21);
  f_a = muDoubleScalarAbs(o[1] - t22_tmp * 0.21);
  a_tmp = o[1] + -(t22_tmp * 0.4);
  g_a = muDoubleScalarAbs(a_tmp);
  b_a_tmp = -o[2] + t21;
  h_a = muDoubleScalarAbs(b_a_tmp + 0.34);
  i_a = muDoubleScalarAbs(o[0] - t30);
  c_a_tmp = t3 * t5;
  j_a = muDoubleScalarAbs(((b_a_tmp + c_a_tmp * 0.21) + t43_tmp * 0.21) + 0.34);
  k_a = muDoubleScalarAbs((a_tmp + t11 * (t17 * 0.21 + t4 * 0.21)) -
                          t45_tmp * 0.21);
  d_a_tmp = -o[0] + t30;
  e_a_tmp = t2 * t5 * t9;
  l_a = muDoubleScalarAbs((d_a_tmp + t11 * (t20 * 0.21 - t8 * 0.21)) +
                          e_a_tmp * 0.21);
  b_a_tmp = (b_a_tmp + t5 * t21) + t43_tmp * 0.4;
  m_a = muDoubleScalarAbs(((b_a_tmp + t12 * (t19 * 0.0607 - t26 * 0.0607)) -
                           t6 * t9 * t10 * 0.0607) +
                          0.34);
  a_tmp = (a_tmp + t45_tmp * -0.4) + t11 * (t17 * 0.4 + t18 * t21);
  n_a = muDoubleScalarAbs((a_tmp + t6 * (t16 * 0.0607 - t27 * 0.0607)) -
                          t12 * (t22 * 0.0607 + t51 * 0.0607));
  d_a_tmp = (d_a_tmp + t5 * t30) + -t11 * (t8 * 0.4 - t20 * 0.4);
  t30 = muDoubleScalarAbs((d_a_tmp + t6 * (t18 * 0.0607 + t24 * 0.0607)) +
                          t12 * (t28 * 0.0607 - t52 * 0.0607));
  t20 = t11 * t49;
  t49 = t12 * (t18 + t24);
  t28_tmp = t6 * (t28 - t52);
  t5 = muDoubleScalarAbs((d_a_tmp + t7 * (t20 * 0.081 + e_a_tmp * 0.081)) -
                         t13 * (t49 * 0.081 - t28_tmp * 0.081));
  t17 = t6 * (t19 + -t26);
  t8 = t9 * t10 * t12;
  t21 =
      muDoubleScalarAbs(((b_a_tmp + t7 * (c_a_tmp * 0.081 + t43_tmp * 0.081)) +
                         t13 * (t17 * 0.081 + t8 * 0.081)) +
                        0.34);
  t2 = t11 * t46;
  t3 = t12 * (t16 + -t27);
  t4 = t6 * (t22 + t51);
  t22_tmp = muDoubleScalarAbs((a_tmp + t7 * (t2 * 0.081 - t45_tmp * 0.081)) -
                              t13 * (t3 * 0.081 + t4 * 0.081));
  t28_tmp = muDoubleScalarAbs((d_a_tmp + t7 * (t20 * 0.126 + e_a_tmp * 0.126)) -
                              t13 * (t49 * 0.126 - t28_tmp * 0.126));
  t8 = muDoubleScalarAbs(((b_a_tmp + t7 * (c_a_tmp * 0.126 + t43_tmp * 0.126)) +
                          t13 * (t17 * 0.126 + t8 * 0.126)) +
                         0.34);
  t4 = muDoubleScalarAbs((a_tmp + t7 * (t2 * 0.126 - t45_tmp * 0.126)) -
                         t13 * (t3 * 0.126 + t4 * 0.126));
  dn1[0] = muDoubleScalarSqrt((a * a + b_a * b_a) + c_a * c_a);
  dn1[1] = muDoubleScalarSqrt((d_a * d_a + e_a * e_a) + f_a * f_a);
  dn1[2] = muDoubleScalarSqrt((g_a * g_a + h_a * h_a) + i_a * i_a);
  dn1[3] = muDoubleScalarSqrt((j_a * j_a + k_a * k_a) + l_a * l_a);
  dn1[4] = muDoubleScalarSqrt((m_a * m_a + n_a * n_a) + t30 * t30);
  dn1[5] = muDoubleScalarSqrt((t5 * t5 + t21 * t21) + t22_tmp * t22_tmp);
  dn1[6] = muDoubleScalarSqrt((t28_tmp * t28_tmp + t8 * t8) + t4 * t4);
}

/* End of code generation (get_dn_1.c) */