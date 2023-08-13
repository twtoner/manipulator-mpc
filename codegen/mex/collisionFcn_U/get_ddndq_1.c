/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * get_ddndq_1.c
 *
 * Code generation for function 'get_ddndq_1'
 *
 */

/* Include files */
#include "get_ddndq_1.h"
#include "collisionFcn_U_data.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"

/* Variable Definitions */
static emlrtRSInfo pe_emlrtRSI = {
    332,                                                          /* lineNo */
    "get_ddndq_1",                                                /* fcnName */
    "G:\\My Drive\\AEROSP 740\\project\\utilities\\get_ddndq_1.m" /* pathName */
};

static emlrtRSInfo re_emlrtRSI = {
    355,                                                          /* lineNo */
    "get_ddndq_1",                                                /* fcnName */
    "G:\\My Drive\\AEROSP 740\\project\\utilities\\get_ddndq_1.m" /* pathName */
};

static emlrtRSInfo af_emlrtRSI = {
    376,                                                          /* lineNo */
    "get_ddndq_1",                                                /* fcnName */
    "G:\\My Drive\\AEROSP 740\\project\\utilities\\get_ddndq_1.m" /* pathName */
};

static emlrtRSInfo bf_emlrtRSI = {
    377,                                                          /* lineNo */
    "get_ddndq_1",                                                /* fcnName */
    "G:\\My Drive\\AEROSP 740\\project\\utilities\\get_ddndq_1.m" /* pathName */
};

static emlrtRSInfo cf_emlrtRSI = {
    378,                                                          /* lineNo */
    "get_ddndq_1",                                                /* fcnName */
    "G:\\My Drive\\AEROSP 740\\project\\utilities\\get_ddndq_1.m" /* pathName */
};

static emlrtRSInfo df_emlrtRSI = {
    379,                                                          /* lineNo */
    "get_ddndq_1",                                                /* fcnName */
    "G:\\My Drive\\AEROSP 740\\project\\utilities\\get_ddndq_1.m" /* pathName */
};

/* Function Definitions */
void get_ddndq_1(const emlrtStack *sp, const real_T q[7], const real_T o[21],
                 real_T D_dn1[49])
{
  emlrtStack st;
  real_T dv[49];
  real_T a;
  real_T a_tmp;
  real_T b_a;
  real_T b_a_tmp;
  real_T b_t317_tmp;
  real_T b_t317_tmp_tmp;
  real_T b_t321_tmp;
  real_T b_t321_tmp_tmp;
  real_T c_a;
  real_T c_a_tmp;
  real_T c_t317_tmp;
  real_T c_t321_tmp_tmp;
  real_T d;
  real_T d1;
  real_T d_a;
  real_T d_a_tmp;
  real_T d_t321_tmp_tmp;
  real_T e_a;
  real_T e_a_tmp;
  real_T e_t321_tmp_tmp;
  real_T f_a;
  real_T f_a_tmp;
  real_T g_a;
  real_T g_a_tmp;
  real_T h_a;
  real_T h_a_tmp;
  real_T i_a;
  real_T i_a_tmp;
  real_T j_a;
  real_T j_a_tmp;
  real_T k_a;
  real_T k_a_tmp;
  real_T t10;
  real_T t11;
  real_T t116;
  real_T t118;
  real_T t12;
  real_T t126;
  real_T t13;
  real_T t130;
  real_T t137;
  real_T t138;
  real_T t14;
  real_T t142;
  real_T t143;
  real_T t146;
  real_T t147;
  real_T t15;
  real_T t162;
  real_T t164;
  real_T t166;
  real_T t172;
  real_T t19;
  real_T t196;
  real_T t197;
  real_T t2;
  real_T t20;
  real_T t200;
  real_T t201;
  real_T t21;
  real_T t219;
  real_T t22;
  real_T t220;
  real_T t228;
  real_T t23;
  real_T t232;
  real_T t25;
  real_T t26;
  real_T t261;
  real_T t263;
  real_T t264_tmp;
  real_T t265;
  real_T t266_tmp;
  real_T t268;
  real_T t27;
  real_T t272;
  real_T t277;
  real_T t278;
  real_T t279_tmp;
  real_T t28;
  real_T t284;
  real_T t286;
  real_T t288;
  real_T t292;
  real_T t293;
  real_T t296;
  real_T t297;
  real_T t298;
  real_T t3;
  real_T t300;
  real_T t300_tmp;
  real_T t301;
  real_T t307;
  real_T t31;
  real_T t312;
  real_T t313;
  real_T t317;
  real_T t317_tmp;
  real_T t317_tmp_tmp;
  real_T t32;
  real_T t321;
  real_T t321_tmp;
  real_T t321_tmp_tmp;
  real_T t322;
  real_T t323;
  real_T t326;
  real_T t329;
  real_T t33;
  real_T t332;
  real_T t333;
  real_T t34;
  real_T t340;
  real_T t341;
  real_T t341_tmp;
  real_T t343;
  real_T t344;
  real_T t345;
  real_T t345_tmp;
  real_T t35;
  real_T t359;
  real_T t36;
  real_T t361;
  real_T t363;
  real_T t37;
  real_T t38;
  real_T t39;
  real_T t4;
  real_T t40;
  real_T t41;
  real_T t42;
  real_T t43;
  real_T t44;
  real_T t49;
  real_T t5;
  real_T t53;
  real_T t54;
  real_T t59;
  real_T t6;
  real_T t60;
  real_T t61;
  real_T t62;
  real_T t63;
  real_T t7;
  real_T t76;
  real_T t8;
  real_T t83_tmp;
  real_T t9;
  real_T t94;
  real_T t95_tmp;
  real_T t95_tmp_tmp;
  real_T t96_tmp;
  real_T t96_tmp_tmp;
  real_T t98;
  int32_T i;
  int32_T i1;
  st.prev = sp;
  st.tls = sp->tls;
  /* GET_DDDQ_1 */
  /*     D_DN1 = GET_DDDQ_1(O1,O2,O3,Q1,Q2,Q3,Q4,Q5,Q6) */
  /*     This function was generated by the Symbolic Math Toolbox version 8.7.
   */
  /*     08-Dec-2021 21:29:56 */
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
  t14 = o[1] * t2;
  t15 = o[0] * t8;
  t19 = t2 * t4;
  t20 = t3 * t5;
  t21 = t2 * t10;
  t22 = t4 * t8;
  t23 = t8 * t10;
  t25 = t3 * 0.4;
  t26 = t3 * 200.0;
  t27 = t3 * 400.0;
  t28 = t9 * 0.4;
  t359 = t8 * t9;
  t31 = t359 * t11;
  t33 = t3 * 4000.0;
  t37 = t2 * t5 * t9;
  t343 = t2 * t9;
  t40 = t343 * t11;
  t41 = t5 * t8 * t9;
  t44 = t4 * t9 * t11;
  t59 = t3 * 0.21;
  t95_tmp_tmp = t3 * t4;
  t95_tmp = t95_tmp_tmp * t11;
  t96_tmp_tmp = t4 * t5;
  t96_tmp = t96_tmp_tmp * t9;
  t32 = t11 * t23;
  t34 = t3 * t19;
  t35 = t3 * t21;
  t36 = t3 * t22;
  t38 = t5 * t21;
  t39 = t3 * t23;
  t42 = t11 * t21;
  t43 = t5 * t23;
  t49 = t2 * t28;
  t53 = t5 * t28;
  t54 = t8 * t28;
  t60 = t19 * t20;
  t62 = t20 * t22;
  t83_tmp = t4 * t11;
  t94 = o[0] + t343 * -0.4;
  t98 = o[1] + t359 * -0.4;
  t126 = t37 * 0.126;
  t130 = t41 * 0.126;
  t143 = t22 * 0.0607;
  t162 = t37 * 0.081;
  t164 = t41 * 0.081;
  t166 = (-o[2] + t25) + 0.34;
  t172 = t31 * 0.0607;
  t201 = t40 * 0.0607;
  t219 = (-o[2] + t59) + 0.34;
  t61 = t11 * t34;
  t63 = t11 * t36;
  t76 = t5 * t49;
  a = muDoubleScalarAbs(t94);
  b_a = muDoubleScalarAbs(t98);
  t116 = t32 * 0.126;
  t118 = t22 + t35;
  t137 = o[0] + -(t343 * 0.21);
  t138 = o[1] + -(t359 * 0.21);
  t142 = t32 * 0.081;
  c_a = muDoubleScalarAbs(t166);
  t196 = t35 * 0.0607;
  t197 = t38 * 0.0607;
  t200 = t39 * 0.0607;
  t220 = t21 * 0.4 + t22 * t25;
  t228 = t60 * 0.0607;
  t232 = t62 * 0.0607;
  d_a = muDoubleScalarAbs(t219);
  t261 = (t3 * 40.0 + t20 * 21.0) + t44 * 21.0;
  t265 = -t11 * (t39 * 0.4 - t19 * 0.4);
  t266_tmp = t34 * 0.4 - t23 * 0.4;
  t268 = (t31 + t38) + t62;
  t284 = (t20 * 0.21 + t44 * 0.21) + t166;
  t317_tmp_tmp = t3 * t11;
  t317_tmp = t317_tmp_tmp * t12;
  b_t317_tmp_tmp = t6 * t9;
  b_t317_tmp = b_t317_tmp_tmp * t10;
  c_t317_tmp = t96_tmp * t12;
  t317 = ((((t33 + t20 * 4000.0) + t317_tmp * 607.0) + t44 * 4000.0) +
          -(b_t317_tmp * 607.0)) +
         -(c_t317_tmp * 607.0);
  t321_tmp = t7 * t20;
  b_t321_tmp = t7 * t44;
  t321_tmp_tmp = t3 * t6;
  b_t321_tmp_tmp = t321_tmp_tmp * t11;
  t312 = b_t321_tmp_tmp * t13;
  c_t321_tmp_tmp = t9 * t10;
  d_t321_tmp_tmp = c_t321_tmp_tmp * t12;
  t333 = d_t321_tmp_tmp * t13;
  e_t321_tmp_tmp = t96_tmp_tmp * t6 * t9;
  t343 = e_t321_tmp_tmp * t13;
  t321 = ((((((t26 + t20 * 200.0) + t321_tmp * 63.0) + t44 * 200.0) +
            b_t321_tmp * 63.0) +
           t312 * 63.0) +
          t333 * 63.0) +
         -(t343 * 63.0);
  t322 = ((((((t27 + t20 * 400.0) + t321_tmp * 81.0) + t44 * 400.0) +
            b_t321_tmp * 81.0) +
           t312 * 81.0) +
          t333 * 81.0) +
         -(t343 * 81.0);
  t359 = t20 * 0.4 + t83_tmp * t28;
  t323 = (((t359 + t166) + t317_tmp * 0.0607) + -(b_t317_tmp * 0.0607)) +
         -(c_t317_tmp * 0.0607);
  t326 = (((((t359 + t321_tmp * 0.126) + b_t321_tmp * 0.126) + t312 * 0.126) +
           t333 * 0.126) +
          t166) +
         -(t343 * 0.126);
  t329 = (((((t359 + t321_tmp * 0.081) + t166) + b_t321_tmp * 0.081) +
           t312 * 0.081) +
          t333 * 0.081) +
         -(t343 * 0.081);
  e_a = muDoubleScalarAbs(t137);
  f_a = muDoubleScalarAbs(t138);
  t146 = t19 + -t39;
  t147 = t23 + -t34;
  t263 = t11 * (t22 * 0.4 + t21 * t25);
  t264_tmp = t12 * t118;
  t272 = (t40 + t60) + -t43;
  t277 = t143 + t196;
  g_a = muDoubleScalarAbs(t284);
  t298 = (t172 + t197) + t232;
  h_a = muDoubleScalarAbs(t323);
  i_a = muDoubleScalarAbs(t326);
  j_a = muDoubleScalarAbs(t329);
  t278 = t19 * 0.0607 + -t200;
  t279_tmp = t6 * t272;
  t288 = (t42 * 0.126 + -t130) + t63 * 0.126;
  t293 = (t42 * 0.081 + -t164) + t63 * 0.081;
  t297 = ((t98 + t42 * 0.21) + -(t41 * 0.21)) + t63 * 0.21;
  t301 = (t201 + -(t43 * 0.0607)) + t228;
  t286 = (t116 + t126) + -(t61 * 0.126);
  t292 = (t142 + t162) + -(t61 * 0.081);
  d = (a * a + b_a * b_a) + c_a * c_a;
  st.site = &pe_emlrtRSI;
  if (d < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &st, &h_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
  }
  d = muDoubleScalarSqrt(d);
  t296 = 1.0 / d;
  a = muDoubleScalarAbs(t297);
  t300_tmp = -o[0] + t49;
  t300 = ((t300_tmp + t32 * 0.21) + t37 * 0.21) + -(t61 * 0.21);
  t359 = t12 * t146;
  t96_tmp_tmp = t6 * t268;
  t307 = t359 * 0.126 + t96_tmp_tmp * 0.126;
  t313 = t359 * 0.081 + t96_tmp_tmp * 0.081;
  a_tmp = ((t300_tmp + t32 * 0.4) + t76) + t61 * -0.4;
  b_a = muDoubleScalarAbs(
      ((((a_tmp + t6 * t143) + t6 * t196) + t12 * t201) + t12 * t43 * -0.0607) +
      t12 * t228);
  b_a_tmp = (((-o[1] + t54) + t8 * t53) + -(t42 * 0.4)) + t63 * -0.4;
  c_a = muDoubleScalarAbs(
      ((((b_a_tmp + t6 * t19 * -0.0607) + t6 * t200) + t12 * t197) +
       t12 * t172) +
      t12 * t232);
  c_a_tmp = t12 * t13;
  d_a_tmp = t6 * t13;
  e_a_tmp = c_a_tmp * t22;
  f_a_tmp = d_a_tmp * t40;
  g_a_tmp = t7 * t61;
  h_a_tmp = c_a_tmp * t35;
  i_a_tmp = d_a_tmp * t43;
  j_a_tmp = d_a_tmp * t60;
  k_a = muDoubleScalarAbs(
      (((((((a_tmp + t7 * t126) + t7 * t116) + -(e_a_tmp * 0.126)) +
          f_a_tmp * 0.126) +
         g_a_tmp * -0.126) +
        -(h_a_tmp * 0.126)) +
       -(i_a_tmp * 0.126)) +
      j_a_tmp * 0.126);
  t359 = c_a_tmp * t19;
  t96_tmp_tmp = t7 * t42;
  t321_tmp = d_a_tmp * t38;
  b_t321_tmp = d_a_tmp * t31;
  t333 = t7 * t63;
  c_a_tmp *= t39;
  k_a_tmp = d_a_tmp * t62;
  t343 = muDoubleScalarAbs(
      (((((((b_a_tmp + t7 * t130) + t359 * 0.126) + t96_tmp_tmp * -0.126) +
          t321_tmp * 0.126) +
         b_t321_tmp * 0.126) +
        t333 * -0.126) +
       -(c_a_tmp * 0.126)) +
      k_a_tmp * 0.126);
  t162 = muDoubleScalarAbs(
      (((((((a_tmp + t7 * t162) + t7 * t142) + -(e_a_tmp * 0.081)) +
          f_a_tmp * 0.081) +
         g_a_tmp * -0.081) +
        -(h_a_tmp * 0.081)) +
       -(i_a_tmp * 0.081)) +
      j_a_tmp * 0.081);
  t321_tmp = muDoubleScalarAbs(
      (((((((b_a_tmp + t7 * t164) + t359 * 0.081) + t96_tmp_tmp * -0.081) +
          t321_tmp * 0.081) +
         b_t321_tmp * 0.081) +
        t333 * -0.081) +
       -(c_a_tmp * 0.081)) +
      k_a_tmp * 0.081);
  b_t321_tmp = muDoubleScalarAbs(t300);
  d = (e_a * e_a + f_a * f_a) + d_a * d_a;
  st.site = &re_emlrtRSI;
  if (d < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &st, &h_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
  }
  d = muDoubleScalarSqrt(d);
  t312 = 1.0 / d;
  t96_tmp_tmp = (t98 + t41 * -0.4) + t11 * t220;
  t332 = (t96_tmp_tmp + t6 * t278) + -(t12 * t298);
  t359 = (t300_tmp + t76) + -t11 * t266_tmp;
  t333 = (t359 + t6 * t277) + t12 * t301;
  t341_tmp = t264_tmp * 0.126 - t279_tmp * 0.126;
  t341 = (t359 + t7 * t286) + -t13 * t341_tmp;
  t345_tmp = t264_tmp * 0.081 - t279_tmp * 0.081;
  t345 = (t359 + t7 * t292) + -t13 * t345_tmp;
  t340 = (t96_tmp_tmp + t7 * t288) + -(t13 * t307);
  t344 = (t96_tmp_tmp + t7 * t293) + -(t13 * t313);
  d = (h_a * h_a + b_a * b_a) + c_a * c_a;
  st.site = &af_emlrtRSI;
  if (d < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &st, &h_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
  }
  d = muDoubleScalarSqrt(d);
  t359 = 1.0 / d;
  d = (i_a * i_a + k_a * k_a) + t343 * t343;
  st.site = &bf_emlrtRSI;
  if (d < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &st, &h_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
  }
  d = muDoubleScalarSqrt(d);
  t361 = 1.0 / d;
  d = (j_a * j_a + t162 * t162) + t321_tmp * t321_tmp;
  st.site = &cf_emlrtRSI;
  if (d < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &st, &h_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
  }
  d = muDoubleScalarSqrt(d);
  t363 = 1.0 / d;
  d = (g_a * g_a + a * a) + b_t321_tmp * b_t321_tmp;
  st.site = &df_emlrtRSI;
  if (d < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &st, &h_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
  }
  d = muDoubleScalarSqrt(d);
  t343 = 1.0 / d;
  dv[0] = 0.0;
  dv[1] = 0.0;
  dv[2] = 0.0;
  dv[3] = 0.0;
  dv[4] = 0.0;
  dv[5] = 0.0;
  dv[6] = 0.0;
  d = t9 * (t14 + -t15);
  dv[7] = d * t312 * -0.21;
  a = t2 * t3;
  d1 = t3 * t8;
  dv[8] =
      t312 * ((t9 * t219 * 21.0 + a * t137 * 21.0) + d1 * t138 * 21.0) * -0.01;
  dv[9] = 0.0;
  dv[10] = 0.0;
  dv[11] = 0.0;
  dv[12] = 0.0;
  dv[13] = 0.0;
  dv[14] = d * t296 * -0.4;
  dv[15] = t296 * ((t9 * t166 * 2.0 + a * t94 * 2.0) + d1 * t98 * 2.0) * -0.2;
  dv[16] = 0.0;
  dv[17] = 0.0;
  dv[18] = 0.0;
  dv[19] = 0.0;
  dv[20] = 0.0;
  d = t5 * t9;
  a = o[1] * t32;
  d1 = o[0] * t42;
  j_a = t9 * t14;
  g_a = t9 * t15;
  c_a = d * t14;
  t296 = d * t15;
  b_a = t95_tmp * t14;
  dv[21] =
      t343 *
      (((((((a * 21.0 + d1 * 21.0) + j_a * 40.0) - g_a * 40.0) + c_a * 21.0) -
         t296 * 21.0) -
        b_a * 21.0) +
       t95_tmp * t15 * 21.0) *
      -0.01;
  dv[22] = t343 *
           ((t284 * ((t28 + d * 0.21) - t95_tmp * 0.21) * 2.0 -
             t2 * t261 * t300 / 50.0) +
            t8 * t261 * t297 / 50.0) *
           -0.5;
  dv[23] = t343 *
           ((t11 * t297 * (t19 * 0.21 - t39 * 0.21) * 2.0 +
             t11 * t300 * (t22 * 0.21 + t35 * 0.21) * 2.0) -
            c_t321_tmp_tmp * t11 * t284 * 0.42) /
           2.0;
  dv[24] = t343 *
           ((t284 * (t11 * t59 - t96_tmp * 0.21) * 2.0 -
             t297 * ((t31 * 0.21 + t38 * 0.21) + t62 * 0.21) * 2.0) +
            t300 * ((t40 * 0.21 - t43 * 0.21) + t60 * 0.21) * 2.0) *
           -0.5;
  dv[25] = 0.0;
  dv[26] = 0.0;
  dv[27] = 0.0;
  d = t321_tmp_tmp * t10;
  d_a = t9 * t11 * t12;
  t264_tmp = t4 * t12;
  t279_tmp = o[0] * t12;
  h_a = o[1] * t12;
  t76 = o[0] * t6;
  i_a = o[1] * t6;
  t164 = t83_tmp * t15;
  dv[28] = t359 *
           (((((((((((((((((a * 4000.0 + d1 * 4000.0) + j_a * 4000.0) -
                          g_a * 4000.0) +
                         t76 * t19 * 607.0) +
                        i_a * t22 * 607.0) -
                       t279_tmp * t38 * 607.0) -
                      h_a * t43 * 607.0) +
                     c_a * 4000.0) -
                    t296 * 4000.0) -
                   b_a * 4000.0) +
                  d * t14 * 607.0) -
                 d * t15 * 607.0) +
                d_a * t14 * 607.0) -
               d_a * t15 * 607.0) +
              t264_tmp * t14 * t20 * 607.0) -
             t264_tmp * t15 * t20 * 607.0) +
            t164 * t33) *
           -0.0001;
  t130 = (t28 + t53) + t95_tmp * -0.4;
  dv[29] =
      t359 *
      ((t323 *
            (((t130 + d * 0.0607) + d_a * 0.0607) + t264_tmp * t20 * 0.0607) *
            2.0 -
        t2 * t317 * t333 / 5000.0) +
       t8 * t317 * t332 / 5000.0) *
      -0.5;
  d = t5 * t12;
  d_a = t4 * t6;
  t142 = t10 * t11;
  dv[30] =
      t359 *
      ((t332 *
            ((t11 * t146 * -0.4 + t6 * (t21 * 0.0607 + t36 * 0.0607)) +
             d * t146 * 0.0607) *
            2.0 +
        t333 *
            ((-t263 + t6 * (t23 * 0.0607 - t34 * 0.0607)) + d * t118 * 0.0607) *
            2.0) +
       t9 * t323 * ((d_a * 607.0 + t142 * 4000.0) - t5 * t10 * t12 * 607.0) /
           5000.0) *
      -0.5;
  d = t11 * t54 + t5 * t220;
  dv[31] = t359 *
           ((t323 *
                 (((t4 * t53 - t317_tmp_tmp * 0.4) + t12 * t20 * 0.0607) +
                  t12 * t44 * 0.0607) *
                 2.0 -
             t333 *
                 ((t40 * 0.4 -
                   t12 * ((t32 * 0.0607 + t37 * 0.0607) - t61 * 0.0607)) +
                  t5 * t266_tmp) *
                 2.0) +
            t332 * (d + t12 * ((t41 * -0.0607 + t42 * 0.0607) + t63 * 0.0607)) *
                2.0) /
           2.0;
  dv[32] = t359 *
           ((t332 * (t12 * t278 + t6 * t298) * 2.0 +
             t333 * (t12 * t277 - t6 * t301) * 2.0) -
            t323 *
                ((b_t321_tmp_tmp * 0.0607 + d_t321_tmp_tmp * 0.0607) -
                 e_t321_tmp_tmp * 0.0607) *
                2.0) *
           -0.5;
  dv[33] = 0.0;
  dv[34] = 0.0;
  t300_tmp = t5 * t7 * t9;
  k_a = t95_tmp_tmp * t7 * t11;
  f_a = t3 * t10 * t12 * t13;
  t126 = b_t317_tmp_tmp * t11 * t13;
  d_a *= t13;
  t116 = o[1] * t7 * t32;
  e_a = o[0] * t7 * t42;
  t279_tmp = t279_tmp * t13 * t19;
  h_a = h_a * t13 * t22;
  t76 = t76 * t13 * t38;
  i_a = i_a * t13 * t43;
  t232 = t300_tmp * t14;
  t143 = t300_tmp * t15;
  t201 = k_a * t14;
  t172 = k_a * t15;
  b_a_tmp = f_a * t14;
  t162 = f_a * t15;
  a_tmp = t126 * t14;
  t332 = t126 * t15;
  t200 = d_a * t14 * t20;
  t197 = d_a * t15 * t20;
  dv[35] = t363 *
           (((((((((((((((((((((((a * 400.0 + d1 * 400.0) + j_a * 400.0) -
                                g_a * 400.0) +
                               t116 * 81.0) +
                              e_a * 81.0) +
                             c_a * 400.0) -
                            t296 * 400.0) -
                           t279_tmp * 81.0) -
                          h_a * 81.0) -
                         t76 * 81.0) -
                        i_a * 81.0) -
                       b_a * 400.0) +
                      t232 * 81.0) -
                     t143 * 81.0) +
                    t164 * t27) -
                   t201 * 81.0) +
                  t172 * 81.0) -
                 b_a_tmp * 81.0) +
                t162 * 81.0) +
               a_tmp * 81.0) -
              t332 * 81.0) +
             t200 * 81.0) -
            t197 * 81.0) *
           -0.001;
  d_a *= t20;
  dv[36] = t363 *
           ((t329 *
                 (((((t130 + t300_tmp * 0.081) - k_a * 0.081) - f_a * 0.081) +
                   t126 * 0.081) +
                  d_a * 0.081) *
                 2.0 -
             t2 * t322 * t345 / 500.0) +
            t8 * t322 * t344 / 500.0) *
           -0.5;
  i_a_tmp = t5 * t6;
  j_a_tmp = t7 * t11;
  t196 = t12 * t147;
  t228 = i_a_tmp * t118;
  c_a_tmp = j_a_tmp * t118;
  f_a_tmp = t12 * (t21 + t36);
  g_a_tmp = i_a_tmp * t146;
  j_a_tmp *= t146;
  h_a_tmp = t7 * t10 * t11;
  t264_tmp *= t13;
  i_a_tmp = i_a_tmp * t10 * t13;
  dv[37] =
      t363 *
      ((t345 *
            ((t263 + t13 * (t196 * 0.081 - t228 * 0.081)) + c_a_tmp * 0.081) *
            2.0 +
        t344 *
            ((t265 + t13 * (f_a_tmp * 0.081 - g_a_tmp * 0.081)) +
             j_a_tmp * 0.081) *
            2.0) -
       t9 * t329 *
           (((t142 * 400.0 + h_a_tmp * 81.0) - t264_tmp * 81.0) -
            i_a_tmp * 81.0) /
           500.0) /
      2.0;
  k_a_tmp = t11 * t25 + t96_tmp * -0.4;
  e_a_tmp = d_a_tmp * (t20 + t44);
  t333 = d_a_tmp * ((t42 + t63) + -t41);
  t312 = t11 * t49 + -(t5 * t147 * 0.4);
  b_t321_tmp = d_a_tmp * ((t32 + t37) + -t61);
  dv[38] = t363 *
           ((t329 *
                 ((k_a_tmp + t7 * (t317_tmp_tmp * 0.081 - t96_tmp * 0.081)) -
                  e_a_tmp * 0.081) *
                 2.0 -
             t344 *
                 ((d + t7 * ((t31 * 0.081 + t38 * 0.081) + t62 * 0.081)) +
                  t333 * 0.081) *
                 2.0) +
            t345 *
                ((t312 + t7 * ((t40 * 0.081 - t43 * 0.081) + t60 * 0.081)) -
                 b_t321_tmp * 0.081) *
                2.0) *
           -0.5;
  t321_tmp = t6 * t118;
  t96_tmp_tmp = t12 * t272;
  t359 = t6 * t146;
  t343 = t12 * t268;
  dv[39] = t363 *
           ((t13 * t345 * (t321_tmp * 0.081 + t96_tmp_tmp * 0.081) * 2.0 +
             t13 * t344 * (t359 * 0.081 - t343 * 0.081) * 2.0) -
            t13 * t329 *
                ((b_t317_tmp * 0.081 - t317_tmp * 0.081) + c_t317_tmp * 0.081) *
                2.0) *
           -0.5;
  dv[40] = t363 *
           ((t344 * (t13 * t293 + t7 * t313) * 2.0 -
             t329 *
                 (t7 * ((b_t321_tmp_tmp * 0.081 + d_t321_tmp_tmp * 0.081) -
                        e_t321_tmp_tmp * 0.081) -
                  t13 * (t20 * 0.081 + t44 * 0.081)) *
                 2.0) +
            t345 * (t13 * t292 + t7 * t345_tmp) * 2.0) *
           -0.5;
  dv[41] = 0.0;
  dv[42] = t361 *
           (((((((((((((((((((((((a * 200.0 + d1 * 200.0) + j_a * 200.0) -
                                g_a * 200.0) +
                               t116 * 63.0) +
                              e_a * 63.0) +
                             c_a * 200.0) -
                            t296 * 200.0) -
                           t279_tmp * 63.0) -
                          h_a * 63.0) -
                         t76 * 63.0) -
                        i_a * 63.0) -
                       b_a * 200.0) +
                      t232 * 63.0) -
                     t143 * 63.0) +
                    t164 * t26) -
                   t201 * 63.0) +
                  t172 * 63.0) -
                 b_a_tmp * 63.0) +
                t162 * 63.0) +
               a_tmp * 63.0) -
              t332 * 63.0) +
             t200 * 63.0) -
            t197 * 63.0) *
           -0.002;
  dv[43] = t361 *
           ((t326 *
                 (((((t130 + t300_tmp * 0.126) - k_a * 0.126) - f_a * 0.126) +
                   t126 * 0.126) +
                  d_a * 0.126) *
                 2.0 -
             t2 * t321 * t341 / 250.0) +
            t8 * t321 * t340 / 250.0) *
           -0.5;
  dv[44] =
      t361 *
      ((t341 *
            ((t263 + t13 * (t196 * 0.126 - t228 * 0.126)) + c_a_tmp * 0.126) *
            2.0 +
        t340 *
            ((t265 + t13 * (f_a_tmp * 0.126 - g_a_tmp * 0.126)) +
             j_a_tmp * 0.126) *
            2.0) -
       t9 * t326 *
           (((t142 * 200.0 + h_a_tmp * 63.0) - t264_tmp * 63.0) -
            i_a_tmp * 63.0) /
           250.0) /
      2.0;
  dv[45] = t361 *
           ((t326 *
                 ((k_a_tmp + t7 * (t317_tmp_tmp * 0.126 - t96_tmp * 0.126)) -
                  e_a_tmp * 0.126) *
                 2.0 -
             t340 *
                 ((d + t7 * ((t31 * 0.126 + t38 * 0.126) + t62 * 0.126)) +
                  t333 * 0.126) *
                 2.0) +
            t341 *
                ((t312 + t7 * ((t40 * 0.126 - t43 * 0.126) + t60 * 0.126)) -
                 b_t321_tmp * 0.126) *
                2.0) *
           -0.5;
  dv[46] = t361 *
           ((t13 * t341 * (t321_tmp * 0.126 + t96_tmp_tmp * 0.126) * 2.0 +
             t13 * t340 * (t359 * 0.126 - t343 * 0.126) * 2.0) -
            t13 * t326 *
                ((b_t317_tmp * 0.126 - t317_tmp * 0.126) + c_t317_tmp * 0.126) *
                2.0) *
           -0.5;
  dv[47] = t361 *
           ((t340 * (t13 * t288 + t7 * t307) * 2.0 -
             t326 *
                 (t7 * ((b_t321_tmp_tmp * 0.126 + d_t321_tmp_tmp * 0.126) -
                        e_t321_tmp_tmp * 0.126) -
                  t13 * (t20 * 0.126 + t44 * 0.126)) *
                 2.0) +
            t341 * (t13 * t286 + t7 * t341_tmp) * 2.0) *
           -0.5;
  dv[48] = 0.0;
  for (i = 0; i < 7; i++) {
    for (i1 = 0; i1 < 7; i1++) {
      D_dn1[i1 + 7 * i] = dv[i + 7 * i1];
    }
  }
}

/* End of code generation (get_ddndq_1.c) */
