/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * get_dn_ddndq.c
 *
 * Code generation for function 'get_dn_ddndq'
 *
 */

/* Include files */
#include "get_dn_ddndq.h"
#include "dot.h"
#include "find.h"
#include "get_ddndq_1.h"
#include "get_ddndq_2.h"
#include "get_ddndq_3.h"
#include "get_dn_1.h"
#include "get_dn_2.h"
#include "get_dn_3.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo o_emlrtRSI = {
    44,                                                            /* lineNo */
    "get_dn_ddndq",                                                /* fcnName */
    "G:\\My Drive\\AEROSP 740\\project\\utilities\\get_dn_ddndq.m" /* pathName
                                                                    */
};

static emlrtRSInfo p_emlrtRSI = {
    45,                                                            /* lineNo */
    "get_dn_ddndq",                                                /* fcnName */
    "G:\\My Drive\\AEROSP 740\\project\\utilities\\get_dn_ddndq.m" /* pathName
                                                                    */
};

static emlrtRSInfo q_emlrtRSI = {
    48,                                                            /* lineNo */
    "get_dn_ddndq",                                                /* fcnName */
    "G:\\My Drive\\AEROSP 740\\project\\utilities\\get_dn_ddndq.m" /* pathName
                                                                    */
};

static emlrtRSInfo s_emlrtRSI = {
    50,                                                            /* lineNo */
    "get_dn_ddndq",                                                /* fcnName */
    "G:\\My Drive\\AEROSP 740\\project\\utilities\\get_dn_ddndq.m" /* pathName
                                                                    */
};

static emlrtRSInfo t_emlrtRSI = {
    51,                                                            /* lineNo */
    "get_dn_ddndq",                                                /* fcnName */
    "G:\\My Drive\\AEROSP 740\\project\\utilities\\get_dn_ddndq.m" /* pathName
                                                                    */
};

static emlrtECInfo g_emlrtECI = {
    -1,                                                            /* nDims */
    54,                                                            /* lineNo */
    5,                                                             /* colNo */
    "get_dn_ddndq",                                                /* fName */
    "G:\\My Drive\\AEROSP 740\\project\\utilities\\get_dn_ddndq.m" /* pName */
};

static emlrtECInfo h_emlrtECI = {
    -1,                                                            /* nDims */
    55,                                                            /* lineNo */
    5,                                                             /* colNo */
    "get_dn_ddndq",                                                /* fName */
    "G:\\My Drive\\AEROSP 740\\project\\utilities\\get_dn_ddndq.m" /* pName */
};

/* Function Definitions */
void get_dn_ddndq(const emlrtStack *sp, const real_T q[7], const real_T O[3],
                  real_T dn[7], real_T ddndq[49], real_T h[7])
{
  emlrtStack st;
  real_T T[128];
  real_T ddndq1[49];
  real_T ddndq2[49];
  real_T b_o[21];
  real_T b_y1[21];
  real_T o[21];
  real_T dn1[7];
  real_T dn2[7];
  real_T T_tmp;
  real_T T_tmp_tmp;
  real_T b_T_tmp;
  real_T b_T_tmp_tmp;
  real_T c_T_tmp;
  real_T d_T_tmp;
  real_T e_T_tmp;
  real_T f_T_tmp;
  real_T g_T_tmp;
  real_T h_T_tmp;
  real_T i_T_tmp;
  real_T j_T_tmp;
  real_T k_T_tmp;
  real_T l_T_tmp;
  real_T v_cos_0_tmp;
  real_T v_cos_1_tmp;
  real_T v_cos_2_tmp;
  real_T v_cos_3_tmp;
  real_T v_cos_4_tmp;
  real_T v_cos_5_tmp;
  real_T v_cos_tmp;
  real_T v_sin_2_tmp;
  real_T v_sin_3_tmp;
  real_T v_sin_4_tmp;
  real_T v_sin_5_tmp;
  real_T v_sin_tmp;
  real_T v_sub_0;
  real_T v_sum;
  real_T v_sum_0;
  real_T v_sum_0_tmp;
  real_T v_sum_1;
  real_T v_sum_10;
  real_T v_sum_11;
  real_T v_sum_3;
  real_T v_sum_4;
  real_T v_sum_5;
  real_T v_sum_6;
  real_T v_sum_7;
  real_T v_sum_8;
  real_T v_sum_9;
  int32_T i1_data[7];
  int32_T i2_data[7];
  int32_T tmp_data[7];
  int32_T iv[2];
  int32_T iv1[2];
  int32_T tmp_size[2];
  int32_T i;
  int32_T iy;
  int32_T m;
  int32_T s;
  int32_T work_tmp;
  boolean_T b_h[7];
  st.prev = sp;
  st.tls = sp->tls;
  /*  T = getTransforms_iiwa7_mex(q); */
  /*  eef_corr = [0, 0, -1, 0;  */
  /*              0, 1, 0, 0;  */
  /*              1, 0, 0, 0;  */
  /*              0, 0, 0, 1]; */
  /*   */
  /*          return_value = fk_world_iiwa_link_1_xyzo(q) */
  /*      fk_world_iiwa_link_1_xyzo */
  /*       */
  /*      Description */
  /*      ----------- */
  /*       */
  /*      Computes  the  forward  kinematics  from  the  link world to the link
   */
  /*      iiwa_link_1.  The  result  is  returned  as  a  (4  x  4)  double  in
   */
  /*      homogeneous  coordinates,  giving the position and the orientation of
   */
  /*      iiwa_link_1 in the world frame. */
  /*       */
  /*      Parameters */
  /*      ---------- */
  /*       */
  /*      q : double */
  /*          Vector of variables where : */
  /*              - q(1) = theta_iiwa_joint_1 : */
  /*                    Rotation  value  (in  radians)  around the iiwa_joint_1
   */
  /*                    joint axis. */
  /*       */
  /*       */
  v_cos_tmp = muDoubleScalarCos(q[0]);
  v_sum_3 = muDoubleScalarSin(q[0]);
  /*  Returned Matrix */
  T[0] = v_cos_tmp;
  T[4] = -v_sum_3;
  T[8] = 0.0;
  T[12] = 0.0;
  T[1] = v_sum_3;
  T[5] = v_cos_tmp;
  T[9] = 0.0;
  T[13] = 0.0;
  /*   */
  /*          return_value = fk_world_iiwa_link_2_xyzo(q) */
  /*      fk_world_iiwa_link_2_xyzo */
  /*       */
  /*      Description */
  /*      ----------- */
  /*       */
  /*      Computes  the  forward  kinematics  from  the  link world to the link
   */
  /*      iiwa_link_2.  The  result  is  returned  as  a  (4  x  4)  double  in
   */
  /*      homogeneous  coordinates,  giving the position and the orientation of
   */
  /*      iiwa_link_2 in the world frame. */
  /*       */
  /*      Parameters */
  /*      ---------- */
  /*       */
  /*      q : double */
  /*          Vector of variables where : */
  /*              - q(1) = theta_iiwa_joint_1 : */
  /*                    Rotation  value  (in  radians)  around the iiwa_joint_1
   */
  /*                    joint axis. */
  /*              - q(2) = theta_iiwa_joint_2 : */
  /*                    Rotation  value  (in  radians)  around the iiwa_joint_2
   */
  /*                    joint axis. */
  /*       */
  /*       */
  v_cos_0_tmp = muDoubleScalarCos(q[1]);
  v_sin_tmp = muDoubleScalarSin(q[1]);
  /*  Returned Matrix */
  T[16] = -v_cos_tmp * v_cos_0_tmp;
  T_tmp = v_sin_tmp * v_cos_tmp;
  T[20] = T_tmp;
  T[24] = -v_sum_3;
  T[28] = 0.0;
  T[17] = -v_sum_3 * v_cos_0_tmp;
  b_T_tmp = v_sum_3 * v_sin_tmp;
  T[21] = b_T_tmp;
  T[25] = v_cos_tmp;
  T[29] = 0.0;
  T[18] = v_sin_tmp;
  T[22] = v_cos_0_tmp;
  T[26] = 0.0;
  T[30] = 0.34;
  /*   */
  /*          return_value = fk_world_iiwa_link_3_xyzo(q) */
  /*      fk_world_iiwa_link_3_xyzo */
  /*       */
  /*      Description */
  /*      ----------- */
  /*       */
  /*      Computes  the  forward  kinematics  from  the  link world to the link
   */
  /*      iiwa_link_3.  The  result  is  returned  as  a  (4  x  4)  double  in
   */
  /*      homogeneous  coordinates,  giving the position and the orientation of
   */
  /*      iiwa_link_3 in the world frame. */
  /*       */
  /*      Parameters */
  /*      ---------- */
  /*       */
  /*      q : double */
  /*          Vector of variables where : */
  /*              - q(1) = theta_iiwa_joint_1 : */
  /*                    Rotation  value  (in  radians)  around the iiwa_joint_1
   */
  /*                    joint axis. */
  /*              - q(2) = theta_iiwa_joint_2 : */
  /*                    Rotation  value  (in  radians)  around the iiwa_joint_2
   */
  /*                    joint axis. */
  /*              - q(3) = theta_iiwa_joint_3 : */
  /*                    Rotation  value  (in  radians)  around the iiwa_joint_3
   */
  /*                    joint axis. */
  /*       */
  /*       */
  v_cos_1_tmp = muDoubleScalarCos(q[2]);
  v_sum_10 = muDoubleScalarSin(q[2]);
  /*  Returned Matrix */
  c_T_tmp = -v_sum_3 * v_sum_10;
  T[32] = c_T_tmp + v_cos_tmp * v_cos_0_tmp * v_cos_1_tmp;
  T_tmp_tmp = v_sum_10 * v_cos_tmp;
  T[36] = -v_sum_3 * v_cos_1_tmp - T_tmp_tmp * v_cos_0_tmp;
  T[40] = T_tmp;
  d_T_tmp = 0.21 * v_sin_tmp * v_cos_tmp;
  T[44] = d_T_tmp;
  e_T_tmp = v_sum_3 * v_cos_0_tmp * v_cos_1_tmp;
  T[33] = e_T_tmp + T_tmp_tmp;
  f_T_tmp = v_cos_tmp * v_cos_1_tmp;
  T[37] = c_T_tmp * v_cos_0_tmp + f_T_tmp;
  T[41] = b_T_tmp;
  c_T_tmp = 0.21 * v_sum_3 * v_sin_tmp;
  T[45] = c_T_tmp;
  g_T_tmp = -v_sin_tmp * v_cos_1_tmp;
  T[34] = g_T_tmp;
  h_T_tmp = v_sin_tmp * v_sum_10;
  T[38] = h_T_tmp;
  T[42] = v_cos_0_tmp;
  T[46] = 0.21 * v_cos_0_tmp + 0.34;
  /*   */
  /*          return_value = fk_world_iiwa_link_4_xyzo(q) */
  /*      fk_world_iiwa_link_4_xyzo */
  /*       */
  /*      Description */
  /*      ----------- */
  /*       */
  /*      Computes  the  forward  kinematics  from  the  link world to the link
   */
  /*      iiwa_link_4.  The  result  is  returned  as  a  (4  x  4)  double  in
   */
  /*      homogeneous  coordinates,  giving the position and the orientation of
   */
  /*      iiwa_link_4 in the world frame. */
  /*       */
  /*      Parameters */
  /*      ---------- */
  /*       */
  /*      q : double */
  /*          Vector of variables where : */
  /*              - q(1) = theta_iiwa_joint_1 : */
  /*                    Rotation  value  (in  radians)  around the iiwa_joint_1
   */
  /*                    joint axis. */
  /*              - q(2) = theta_iiwa_joint_2 : */
  /*                    Rotation  value  (in  radians)  around the iiwa_joint_2
   */
  /*                    joint axis. */
  /*              - q(3) = theta_iiwa_joint_3 : */
  /*                    Rotation  value  (in  radians)  around the iiwa_joint_3
   */
  /*                    joint axis. */
  /*              - q(4) = theta_iiwa_joint_4 : */
  /*                    Rotation  value  (in  radians)  around the iiwa_joint_4
   */
  /*                    joint axis. */
  /*       */
  /*       */
  v_cos_2_tmp = muDoubleScalarCos(q[3]);
  v_sin_2_tmp = muDoubleScalarSin(q[3]);
  v_sum = e_T_tmp + T_tmp_tmp;
  v_sum_0_tmp = -v_sum_3 * v_sum_10 + v_cos_tmp * v_cos_0_tmp * v_cos_1_tmp;
  /*  Returned Matrix */
  b_T_tmp_tmp = v_sin_tmp * v_sin_2_tmp;
  T[48] = v_sum_0_tmp * v_cos_2_tmp + b_T_tmp_tmp * v_cos_tmp;
  T_tmp *= v_cos_2_tmp;
  T[52] = -v_sum_0_tmp * v_sin_2_tmp + T_tmp;
  i_T_tmp = v_sum_3 * v_cos_1_tmp;
  T[56] = i_T_tmp + T_tmp_tmp * v_cos_0_tmp;
  j_T_tmp = 0.4 * v_sin_tmp * v_cos_tmp;
  T[60] = j_T_tmp;
  T[49] = v_sum * v_cos_2_tmp + b_T_tmp * v_sin_2_tmp;
  T[53] = -v_sum * v_sin_2_tmp + b_T_tmp * v_cos_2_tmp;
  f_T_tmp = v_sum_3 * v_sum_10 * v_cos_0_tmp - f_T_tmp;
  T[57] = f_T_tmp;
  k_T_tmp = 0.4 * v_sum_3 * v_sin_tmp;
  T[61] = k_T_tmp;
  g_T_tmp = g_T_tmp * v_cos_2_tmp + v_sin_2_tmp * v_cos_0_tmp;
  T[50] = g_T_tmp;
  l_T_tmp = v_cos_0_tmp * v_cos_2_tmp;
  T[54] = b_T_tmp_tmp * v_cos_1_tmp + l_T_tmp;
  T[58] = -v_sin_tmp * v_sum_10;
  T[62] = 0.4 * v_cos_0_tmp + 0.34;
  /*   */
  /*          return_value = fk_world_iiwa_link_5_xyzo(q) */
  /*      fk_world_iiwa_link_5_xyzo */
  /*       */
  /*      Description */
  /*      ----------- */
  /*       */
  /*      Computes  the  forward  kinematics  from  the  link world to the link
   */
  /*      iiwa_link_5.  The  result  is  returned  as  a  (4  x  4)  double  in
   */
  /*      homogeneous  coordinates,  giving the position and the orientation of
   */
  /*      iiwa_link_5 in the world frame. */
  /*       */
  /*      Parameters */
  /*      ---------- */
  /*       */
  /*      q : double */
  /*          Vector of variables where : */
  /*              - q(1) = theta_iiwa_joint_1 : */
  /*                    Rotation  value  (in  radians)  around the iiwa_joint_1
   */
  /*                    joint axis. */
  /*              - q(2) = theta_iiwa_joint_2 : */
  /*                    Rotation  value  (in  radians)  around the iiwa_joint_2
   */
  /*                    joint axis. */
  /*              - q(3) = theta_iiwa_joint_3 : */
  /*                    Rotation  value  (in  radians)  around the iiwa_joint_3
   */
  /*                    joint axis. */
  /*              - q(4) = theta_iiwa_joint_4 : */
  /*                    Rotation  value  (in  radians)  around the iiwa_joint_4
   */
  /*                    joint axis. */
  /*              - q(5) = theta_iiwa_joint_5 : */
  /*                    Rotation  value  (in  radians)  around the iiwa_joint_5
   */
  /*                    joint axis. */
  /*       */
  /*       */
  v_cos_3_tmp = muDoubleScalarCos(q[4]);
  v_sin_3_tmp = muDoubleScalarSin(q[4]);
  v_sum = e_T_tmp + T_tmp_tmp;
  v_sum_0 = i_T_tmp + T_tmp_tmp * v_cos_0_tmp;
  v_sum_3 = v_sum * v_cos_2_tmp + b_T_tmp * v_sin_2_tmp;
  v_sum_4 = v_sum_0_tmp * v_cos_2_tmp + b_T_tmp_tmp * v_cos_tmp;
  /*  Returned Matrix */
  T[64] = -v_sum_4 * v_cos_3_tmp + v_sum_0 * v_sin_3_tmp;
  T[68] = v_sum_4 * v_sin_3_tmp + v_sum_0 * v_cos_3_tmp;
  T[72] = -v_sum_0_tmp * v_sin_2_tmp + T_tmp;
  T[76] = (-0.21 * v_sum_0_tmp * v_sin_2_tmp + d_T_tmp * v_cos_2_tmp) + j_T_tmp;
  T[65] = -v_sum_3 * v_cos_3_tmp + f_T_tmp * v_sin_3_tmp;
  T[69] = v_sum_3 * v_sin_3_tmp + f_T_tmp * v_cos_3_tmp;
  T[73] = -v_sum * v_sin_2_tmp + b_T_tmp * v_cos_2_tmp;
  T[77] = (-0.21 * v_sum * v_sin_2_tmp + c_T_tmp * v_cos_2_tmp) + k_T_tmp;
  T[66] = -g_T_tmp * v_cos_3_tmp - h_T_tmp * v_sin_3_tmp;
  T[70] = g_T_tmp * v_sin_3_tmp - h_T_tmp * v_cos_3_tmp;
  T[74] = b_T_tmp_tmp * v_cos_1_tmp + l_T_tmp;
  T[78] = ((0.21 * v_sin_tmp * v_sin_2_tmp * v_cos_1_tmp +
            0.21 * v_cos_0_tmp * v_cos_2_tmp) +
           0.4 * v_cos_0_tmp) +
          0.34;
  /*   */
  /*          return_value = fk_world_iiwa_link_6_xyzo(q) */
  /*      fk_world_iiwa_link_6_xyzo */
  /*       */
  /*      Description */
  /*      ----------- */
  /*       */
  /*      Computes  the  forward  kinematics  from  the  link world to the link
   */
  /*      iiwa_link_6.  The  result  is  returned  as  a  (4  x  4)  double  in
   */
  /*      homogeneous  coordinates,  giving the position and the orientation of
   */
  /*      iiwa_link_6 in the world frame. */
  /*       */
  /*      Parameters */
  /*      ---------- */
  /*       */
  /*      q : double */
  /*          Vector of variables where : */
  /*              - q(1) = theta_iiwa_joint_1 : */
  /*                    Rotation  value  (in  radians)  around the iiwa_joint_1
   */
  /*                    joint axis. */
  /*              - q(2) = theta_iiwa_joint_2 : */
  /*                    Rotation  value  (in  radians)  around the iiwa_joint_2
   */
  /*                    joint axis. */
  /*              - q(3) = theta_iiwa_joint_3 : */
  /*                    Rotation  value  (in  radians)  around the iiwa_joint_3
   */
  /*                    joint axis. */
  /*              - q(4) = theta_iiwa_joint_4 : */
  /*                    Rotation  value  (in  radians)  around the iiwa_joint_4
   */
  /*                    joint axis. */
  /*              - q(5) = theta_iiwa_joint_5 : */
  /*                    Rotation  value  (in  radians)  around the iiwa_joint_5
   */
  /*                    joint axis. */
  /*              - q(6) = theta_iiwa_joint_6 : */
  /*                    Rotation  value  (in  radians)  around the iiwa_joint_6
   */
  /*                    joint axis. */
  /*       */
  /*       */
  v_cos_4_tmp = muDoubleScalarCos(q[5]);
  v_sin_4_tmp = muDoubleScalarSin(q[5]);
  v_sum = e_T_tmp + T_tmp_tmp;
  v_sum_0 = i_T_tmp + T_tmp_tmp * v_cos_0_tmp;
  v_sum_1 = b_T_tmp_tmp * v_cos_1_tmp + l_T_tmp;
  v_sum_4 = v_sum * v_cos_2_tmp + b_T_tmp * v_sin_2_tmp;
  v_sum_5 = v_sum_0_tmp * v_cos_2_tmp + b_T_tmp_tmp * v_cos_tmp;
  v_sum_6 = -v_sum * v_sin_2_tmp + b_T_tmp * v_cos_2_tmp;
  v_sum_7 = -v_sum_0_tmp * v_sin_2_tmp + T_tmp;
  v_sub_0 = -g_T_tmp * v_cos_3_tmp - h_T_tmp * v_sin_3_tmp;
  v_sum_8 = -v_sum_4 * v_cos_3_tmp + f_T_tmp * v_sin_3_tmp;
  v_sum_9 = -v_sum_5 * v_cos_3_tmp + v_sum_0 * v_sin_3_tmp;
  /*  Returned Matrix */
  T[80] = v_sum_9 * v_cos_4_tmp + v_sum_7 * v_sin_4_tmp;
  T[84] = -v_sum_9 * v_sin_4_tmp + v_sum_7 * v_cos_4_tmp;
  T[88] = -v_sum_5 * v_sin_3_tmp - v_sum_0 * v_cos_3_tmp;
  T[92] = (((0.0607 * v_sum_5 * v_sin_3_tmp - 0.4 * v_sum_0_tmp * v_sin_2_tmp) +
            0.0607 * v_sum_0 * v_cos_3_tmp) +
           j_T_tmp * v_cos_2_tmp) +
          j_T_tmp;
  T[81] = v_sum_8 * v_cos_4_tmp + v_sum_6 * v_sin_4_tmp;
  T[85] = -v_sum_8 * v_sin_4_tmp + v_sum_6 * v_cos_4_tmp;
  T[89] = -v_sum_4 * v_sin_3_tmp - f_T_tmp * v_cos_3_tmp;
  T[93] = (((0.0607 * v_sum_4 * v_sin_3_tmp + 0.0607 * f_T_tmp * v_cos_3_tmp) -
            0.4 * v_sum * v_sin_2_tmp) +
           k_T_tmp * v_cos_2_tmp) +
          k_T_tmp;
  T[82] = v_sub_0 * v_cos_4_tmp + v_sum_1 * v_sin_4_tmp;
  T[86] = -v_sub_0 * v_sin_4_tmp + v_sum_1 * v_cos_4_tmp;
  T[90] = -g_T_tmp * v_sin_3_tmp + h_T_tmp * v_cos_3_tmp;
  c_T_tmp = 0.4 * v_sin_tmp * v_sin_2_tmp * v_cos_1_tmp;
  d_T_tmp = 0.4 * v_cos_0_tmp * v_cos_2_tmp;
  T[94] = ((((0.0607 * g_T_tmp * v_sin_3_tmp -
              0.0607 * v_sin_tmp * v_sum_10 * v_cos_3_tmp) +
             c_T_tmp) +
            d_T_tmp) +
           0.4 * v_cos_0_tmp) +
          0.34;
  /*   */
  /*          return_value = fk_world_iiwa_link_7_xyzo(q) */
  /*      fk_world_iiwa_link_7_xyzo */
  /*       */
  /*      Description */
  /*      ----------- */
  /*       */
  /*      Computes  the  forward  kinematics  from  the  link world to the link
   */
  /*      iiwa_link_7.  The  result  is  returned  as  a  (4  x  4)  double  in
   */
  /*      homogeneous  coordinates,  giving the position and the orientation of
   */
  /*      iiwa_link_7 in the world frame. */
  /*       */
  /*      Parameters */
  /*      ---------- */
  /*       */
  /*      q : double */
  /*          Vector of variables where : */
  /*              - q(1) = theta_iiwa_joint_1 : */
  /*                    Rotation  value  (in  radians)  around the iiwa_joint_1
   */
  /*                    joint axis. */
  /*              - q(2) = theta_iiwa_joint_2 : */
  /*                    Rotation  value  (in  radians)  around the iiwa_joint_2
   */
  /*                    joint axis. */
  /*              - q(3) = theta_iiwa_joint_3 : */
  /*                    Rotation  value  (in  radians)  around the iiwa_joint_3
   */
  /*                    joint axis. */
  /*              - q(4) = theta_iiwa_joint_4 : */
  /*                    Rotation  value  (in  radians)  around the iiwa_joint_4
   */
  /*                    joint axis. */
  /*              - q(5) = theta_iiwa_joint_5 : */
  /*                    Rotation  value  (in  radians)  around the iiwa_joint_5
   */
  /*                    joint axis. */
  /*              - q(6) = theta_iiwa_joint_6 : */
  /*                    Rotation  value  (in  radians)  around the iiwa_joint_6
   */
  /*                    joint axis. */
  /*              - q(7) = theta_iiwa_joint_7 : */
  /*                    Rotation  value  (in  radians)  around the iiwa_joint_7
   */
  /*                    joint axis. */
  /*       */
  /*       */
  v_cos_5_tmp = muDoubleScalarCos(q[6]);
  v_sin_5_tmp = muDoubleScalarSin(q[6]);
  v_sum = b_T_tmp_tmp * v_cos_1_tmp + l_T_tmp;
  v_sum_0 = e_T_tmp + T_tmp_tmp;
  v_sum_1 = i_T_tmp + T_tmp_tmp * v_cos_0_tmp;
  v_sum_4 =
      -(-v_sin_tmp * v_cos_1_tmp * v_cos_2_tmp + v_sin_2_tmp * v_cos_0_tmp);
  v_sum_6 = -v_sum_0 * v_sin_2_tmp + b_T_tmp * v_cos_2_tmp;
  v_sum_3 = -(v_sum_0 * v_cos_2_tmp + b_T_tmp * v_sin_2_tmp);
  v_sum_7 = -v_sum_0_tmp * v_sin_2_tmp + T_tmp;
  v_sum_8 = v_sum_4 * v_sin_3_tmp + h_T_tmp * v_cos_3_tmp;
  v_sub_0 = v_sum_4 * v_cos_3_tmp - h_T_tmp * v_sin_3_tmp;
  v_sum_5 = -(v_sum_0_tmp * v_cos_2_tmp + b_T_tmp_tmp * v_cos_tmp);
  v_sum_9 = v_sum_3 * v_cos_3_tmp + f_T_tmp * v_sin_3_tmp;
  v_sum_4 = v_sum_3 * v_sin_3_tmp - f_T_tmp * v_cos_3_tmp;
  v_sum_10 = v_sub_0 * v_cos_4_tmp + v_sum * v_sin_4_tmp;
  v_sum_11 = v_sum_5 * v_cos_3_tmp + v_sum_1 * v_sin_3_tmp;
  v_sum_3 = v_sum_5 * v_sin_3_tmp - v_sum_1 * v_cos_3_tmp;
  v_sum_5 = v_sum_9 * v_cos_4_tmp + v_sum_6 * v_sin_4_tmp;
  /*  Returned Matrix */
  g_T_tmp = v_sum_11 * v_cos_4_tmp + v_sum_7 * v_sin_4_tmp;
  T[96] = -g_T_tmp * v_cos_5_tmp + v_sum_3 * v_sin_5_tmp;
  T[100] = g_T_tmp * v_sin_5_tmp + v_sum_3 * v_cos_5_tmp;
  T[104] = -v_sum_11 * v_sin_4_tmp + v_sum_7 * v_cos_4_tmp;
  T[108] = (((-0.081 * v_sum_11 * v_sin_4_tmp + 0.081 * v_sum_7 * v_cos_4_tmp) -
             0.4 * v_sum_0_tmp * v_sin_2_tmp) +
            j_T_tmp * v_cos_2_tmp) +
           j_T_tmp;
  T[97] = -v_sum_5 * v_cos_5_tmp + v_sum_4 * v_sin_5_tmp;
  T[101] = v_sum_5 * v_sin_5_tmp + v_sum_4 * v_cos_5_tmp;
  T[105] = -v_sum_9 * v_sin_4_tmp + v_sum_6 * v_cos_4_tmp;
  T[109] = (((-0.081 * v_sum_9 * v_sin_4_tmp + 0.081 * v_sum_6 * v_cos_4_tmp) -
             0.4 * v_sum_0 * v_sin_2_tmp) +
            k_T_tmp * v_cos_2_tmp) +
           k_T_tmp;
  T[98] = -v_sum_10 * v_cos_5_tmp + v_sum_8 * v_sin_5_tmp;
  T[102] = v_sum_10 * v_sin_5_tmp + v_sum_8 * v_cos_5_tmp;
  T[106] = -v_sub_0 * v_sin_4_tmp + v_sum * v_cos_4_tmp;
  T[110] = ((((-0.081 * v_sub_0 * v_sin_4_tmp + 0.081 * v_sum * v_cos_4_tmp) +
              c_T_tmp) +
             d_T_tmp) +
            0.4 * v_cos_0_tmp) +
           0.34;
  /*  T(:,:,8) = iiwa7_fk.fk_world_iiwa_link_ee_xyzo(q) * eef_corr; */
  /*   */
  /*          return_value = fk_world_iiwa_link_ee_xyzo(q) */
  /*      fk_world_iiwa_link_ee_xyzo */
  /*       */
  /*      Description */
  /*      ----------- */
  /*       */
  /*      Computes  the  forward  kinematics  from  the  link world to the link
   */
  /*      iiwa_link_ee.  The  result  is  returned  as  a  (4  x  4)  double in
   */
  /*      homogeneous  coordinates,  giving the position and the orientation of
   */
  /*      iiwa_link_ee in the world frame. */
  /*       */
  /*      Parameters */
  /*      ---------- */
  /*       */
  /*      q : double */
  /*          Vector of variables where : */
  /*              - q(1) = theta_iiwa_joint_1 : */
  /*                    Rotation  value  (in  radians)  around the iiwa_joint_1
   */
  /*                    joint axis. */
  /*              - q(2) = theta_iiwa_joint_2 : */
  /*                    Rotation  value  (in  radians)  around the iiwa_joint_2
   */
  /*                    joint axis. */
  /*              - q(3) = theta_iiwa_joint_3 : */
  /*                    Rotation  value  (in  radians)  around the iiwa_joint_3
   */
  /*                    joint axis. */
  /*              - q(4) = theta_iiwa_joint_4 : */
  /*                    Rotation  value  (in  radians)  around the iiwa_joint_4
   */
  /*                    joint axis. */
  /*              - q(5) = theta_iiwa_joint_5 : */
  /*                    Rotation  value  (in  radians)  around the iiwa_joint_5
   */
  /*                    joint axis. */
  /*              - q(6) = theta_iiwa_joint_6 : */
  /*                    Rotation  value  (in  radians)  around the iiwa_joint_6
   */
  /*                    joint axis. */
  /*              - q(7) = theta_iiwa_joint_7 : */
  /*                    Rotation  value  (in  radians)  around the iiwa_joint_7
   */
  /*                    joint axis. */
  /*       */
  /*       */
  v_sum = b_T_tmp_tmp * v_cos_1_tmp + l_T_tmp;
  v_sum_0 = e_T_tmp + T_tmp_tmp;
  v_sum_1 = i_T_tmp + T_tmp_tmp * v_cos_0_tmp;
  v_sum_4 =
      -(-v_sin_tmp * v_cos_1_tmp * v_cos_2_tmp + v_sin_2_tmp * v_cos_0_tmp);
  v_sum_6 = -v_sum_0 * v_sin_2_tmp + b_T_tmp * v_cos_2_tmp;
  v_sum_3 = -(v_sum_0 * v_cos_2_tmp + b_T_tmp * v_sin_2_tmp);
  v_sum_7 = -v_sum_0_tmp * v_sin_2_tmp + T_tmp;
  v_sum_8 = v_sum_4 * v_sin_3_tmp + h_T_tmp * v_cos_3_tmp;
  v_sub_0 = v_sum_4 * v_cos_3_tmp - h_T_tmp * v_sin_3_tmp;
  v_sum_5 = -(v_sum_0_tmp * v_cos_2_tmp + b_T_tmp_tmp * v_cos_tmp);
  v_sum_9 = v_sum_3 * v_cos_3_tmp + f_T_tmp * v_sin_3_tmp;
  v_sum_4 = v_sum_3 * v_sin_3_tmp - f_T_tmp * v_cos_3_tmp;
  v_sum_10 = v_sub_0 * v_cos_4_tmp + v_sum * v_sin_4_tmp;
  v_sum_11 = v_sum_5 * v_cos_3_tmp + v_sum_1 * v_sin_3_tmp;
  v_sum_3 = v_sum_5 * v_sin_3_tmp - v_sum_1 * v_cos_3_tmp;
  v_sum_5 = v_sum_9 * v_cos_4_tmp + v_sum_6 * v_sin_4_tmp;
  /*  Returned Matrix */
  T_tmp = v_sum_11 * v_cos_4_tmp + v_sum_7 * v_sin_4_tmp;
  T[112] = -T_tmp * v_cos_5_tmp + v_sum_3 * v_sin_5_tmp;
  T[116] = T_tmp * v_sin_5_tmp + v_sum_3 * v_cos_5_tmp;
  T[120] = -v_sum_11 * v_sin_4_tmp + v_sum_7 * v_cos_4_tmp;
  T[124] = (((-0.126 * v_sum_11 * v_sin_4_tmp + 0.126 * v_sum_7 * v_cos_4_tmp) -
             0.4 * v_sum_0_tmp * v_sin_2_tmp) +
            j_T_tmp * v_cos_2_tmp) +
           j_T_tmp;
  T[113] = -v_sum_5 * v_cos_5_tmp + v_sum_4 * v_sin_5_tmp;
  T[117] = v_sum_5 * v_sin_5_tmp + v_sum_4 * v_cos_5_tmp;
  T[121] = -v_sum_9 * v_sin_4_tmp + v_sum_6 * v_cos_4_tmp;
  T[125] = (((-0.126 * v_sum_9 * v_sin_4_tmp + 0.126 * v_sum_6 * v_cos_4_tmp) -
             0.4 * v_sum_0 * v_sin_2_tmp) +
            k_T_tmp * v_cos_2_tmp) +
           k_T_tmp;
  T[114] = -v_sum_10 * v_cos_5_tmp + v_sum_8 * v_sin_5_tmp;
  T[118] = v_sum_10 * v_sin_5_tmp + v_sum_8 * v_cos_5_tmp;
  T[122] = -v_sub_0 * v_sin_4_tmp + v_sum * v_cos_4_tmp;
  T[126] = ((((-0.126 * v_sub_0 * v_sin_4_tmp + 0.126 * v_sum * v_cos_4_tmp) +
              c_T_tmp) +
             d_T_tmp) +
            0.4 * v_cos_0_tmp) +
           0.34;
  T[2] = 0.0;
  T[3] = 0.0;
  T[19] = 0.0;
  T[35] = 0.0;
  T[51] = 0.0;
  T[67] = 0.0;
  T[83] = 0.0;
  T[99] = 0.0;
  T[115] = 0.0;
  T[6] = 0.0;
  T[7] = 0.0;
  T[23] = 0.0;
  T[39] = 0.0;
  T[55] = 0.0;
  T[71] = 0.0;
  T[87] = 0.0;
  T[103] = 0.0;
  T[119] = 0.0;
  T[10] = 1.0;
  T[11] = 0.0;
  T[27] = 0.0;
  T[43] = 0.0;
  T[59] = 0.0;
  T[75] = 0.0;
  T[91] = 0.0;
  T[107] = 0.0;
  T[123] = 0.0;
  T[14] = 0.15;
  T[15] = 1.0;
  T[31] = 1.0;
  T[47] = 1.0;
  T[63] = 1.0;
  T[79] = 1.0;
  T[95] = 1.0;
  T[111] = 1.0;
  T[127] = 1.0;
  iy = 0;
  for (s = 0; s < 3; s++) {
    v_sum_3 = T[(s % 3 + ((s / 3) << 4)) + 12];
    for (m = 0; m < 7; m++) {
      v_sum_5 = v_sum_3;
      work_tmp = (s + m * 3) + 3;
      v_sum_3 = T[(work_tmp % 3 + ((work_tmp / 3) << 4)) + 12];
      b_y1[iy + m * 3] = v_sum_3 - v_sum_5;
    }
    iy++;
  }
  /*  dn = zeros(7,no); */
  /*  ddndq = zeros(7,7,no); */
  /*  for i = 1:no */
  /*      o = O(:,i); */
  /*      hh = dot(o - p17, pjj) ./ dot(pjj, pjj); */
  /*      dn_ = get_dn_3_mex(q, o);        % case 3 as default */
  /*      ddndq_ = get_ddndq_3_mex(q, o); */
  /*      i1 = find(hh > 1); */
  /*      i2 = find(hh < 0); */
  /*      dn1 = get_dn_1_mex(q, o); */
  /*      dn2 = get_dn_2_mex(q, o); */
  /*      ddndq1 = get_ddndq_1_mex(q, o); */
  /*      ddndq2 = get_ddndq_2_mex(q, o); */
  /*      dn_(i1) = dn1(i1); */
  /*      dn_(i2) = dn2(i2); */
  /*      ddndq_(:,i1) = ddndq1(:,i1); */
  /*      ddndq_(:,i2) = ddndq2(:,i2); */
  /*      dn((i-1)*n+1 : i*n) = dn_; */
  /*  %     dn(:,i) = dn_; */
  /*  %     ddndq(:,:,i) = ddndq_; */
  /*      ddndq((i-1)*n+1 : i*n, :) = ddndq_; */
  /*      h(i,:) = hh; */
  /*  end */
  v_sum_3 = O[0];
  v_sum_5 = O[1];
  v_sum_4 = O[2];
  for (work_tmp = 0; work_tmp < 7; work_tmp++) {
    iy = work_tmp * 3;
    b_o[iy] = v_sum_3;
    b_o[iy + 1] = v_sum_5;
    b_o[iy + 2] = v_sum_4;
    iy = work_tmp << 4;
    o[3 * work_tmp] = v_sum_3 - T[iy + 12];
    o[3 * work_tmp + 1] = v_sum_5 - T[iy + 13];
    o[3 * work_tmp + 2] = v_sum_4 - T[iy + 14];
  }
  dot(o, b_y1, h);
  dot(b_y1, b_y1, dn1);
  for (m = 0; m < 7; m++) {
    h[m] /= dn1[m];
  }
  st.site = &o_emlrtRSI;
  get_dn_3(&st, q, b_o, dn);
  /*  case 3 as default */
  st.site = &p_emlrtRSI;
  get_ddndq_3(&st, q, b_o, ddndq);
  for (m = 0; m < 7; m++) {
    b_h[m] = (h[m] > 1.0);
  }
  eml_find(b_h, tmp_data, tmp_size);
  iy = tmp_size[1];
  work_tmp = tmp_size[1];
  if (0 <= work_tmp - 1) {
    memcpy(&i1_data[0], &tmp_data[0], work_tmp * sizeof(int32_T));
  }
  for (m = 0; m < 7; m++) {
    b_h[m] = (h[m] < 0.0);
  }
  eml_find(b_h, tmp_data, tmp_size);
  s = tmp_size[1];
  work_tmp = tmp_size[1];
  if (0 <= work_tmp - 1) {
    memcpy(&i2_data[0], &tmp_data[0], work_tmp * sizeof(int32_T));
  }
  st.site = &q_emlrtRSI;
  get_dn_1(&st, q, b_o, dn1);
  get_dn_2(q, b_o, dn2);
  st.site = &s_emlrtRSI;
  get_ddndq_1(&st, q, b_o, ddndq1);
  st.site = &t_emlrtRSI;
  get_ddndq_2(&st, q, b_o, ddndq2);
  if (0 <= iy - 1) {
    memcpy(&tmp_data[0], &i1_data[0], iy * sizeof(int32_T));
  }
  for (m = 0; m < iy; m++) {
    i = tmp_data[m];
    dn[i - 1] = dn1[i - 1];
  }
  if (0 <= s - 1) {
    memcpy(&tmp_data[0], &i2_data[0], s * sizeof(int32_T));
  }
  work_tmp = tmp_size[1];
  for (m = 0; m < work_tmp; m++) {
    i = tmp_data[m];
    dn[i - 1] = dn2[i - 1];
  }
  for (m = 0; m < iy; m++) {
    tmp_data[m] = i1_data[m] - 1;
  }
  iv[0] = 7;
  iv[1] = iy;
  iv1[0] = 7;
  iv1[1] = iy;
  emlrtSubAssignSizeCheckR2012b(&iv[0], 2, &iv1[0], 2, &g_emlrtECI,
                                (emlrtCTX)sp);
  for (m = 0; m < iy; m++) {
    for (i = 0; i < 7; i++) {
      ddndq[i + 7 * tmp_data[m]] = ddndq1[i + 7 * (i1_data[m] - 1)];
    }
  }
  for (m = 0; m < s; m++) {
    tmp_data[m] = i2_data[m] - 1;
  }
  iv[0] = 7;
  iv[1] = tmp_size[1];
  iv1[0] = 7;
  iv1[1] = tmp_size[1];
  emlrtSubAssignSizeCheckR2012b(&iv[0], 2, &iv1[0], 2, &h_emlrtECI,
                                (emlrtCTX)sp);
  for (m = 0; m < s; m++) {
    for (i = 0; i < 7; i++) {
      ddndq[i + 7 * tmp_data[m]] = ddndq2[i + 7 * (i2_data[m] - 1)];
    }
  }
}

/* End of code generation (get_dn_ddndq.c) */
