/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * get_ddndq_3.c
 *
 * Code generation for function 'get_ddndq_3'
 *
 */

/* Include files */
#include "get_ddndq_3.h"
#include "collisionFcn_U_data.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"

/* Variable Definitions */
static emlrtRSInfo ac_emlrtRSI = {
    519,                                                          /* lineNo */
    "get_ddndq_3",                                                /* fcnName */
    "G:\\My Drive\\AEROSP 740\\project\\utilities\\get_ddndq_3.m" /* pathName */
};

static emlrtRSInfo gc_emlrtRSI = {
    1461,                                                         /* lineNo */
    "ft_1",                                                       /* fcnName */
    "G:\\My Drive\\AEROSP 740\\project\\utilities\\get_ddndq_3.m" /* pathName */
};

static emlrtRSInfo qc_emlrtRSI = {
    1941,                                                         /* lineNo */
    "ft_2",                                                       /* fcnName */
    "G:\\My Drive\\AEROSP 740\\project\\utilities\\get_ddndq_3.m" /* pathName */
};

static emlrtRSInfo md_emlrtRSI = {
    2166,                                                         /* lineNo */
    "ft_2",                                                       /* fcnName */
    "G:\\My Drive\\AEROSP 740\\project\\utilities\\get_ddndq_3.m" /* pathName */
};

static emlrtRSInfo od_emlrtRSI = {
    2170,                                                         /* lineNo */
    "ft_2",                                                       /* fcnName */
    "G:\\My Drive\\AEROSP 740\\project\\utilities\\get_ddndq_3.m" /* pathName */
};

static emlrtRSInfo pd_emlrtRSI = {
    2172,                                                         /* lineNo */
    "ft_2",                                                       /* fcnName */
    "G:\\My Drive\\AEROSP 740\\project\\utilities\\get_ddndq_3.m" /* pathName */
};

static emlrtRSInfo qd_emlrtRSI = {
    2173,                                                         /* lineNo */
    "ft_2",                                                       /* fcnName */
    "G:\\My Drive\\AEROSP 740\\project\\utilities\\get_ddndq_3.m" /* pathName */
};

/* Function Declarations */
static void ft_1(const emlrtStack *sp, const real_T ct[439], real_T D_dn3[49]);

static void ft_2(const emlrtStack *sp, const real_T ct[315], real_T D_dn3[49]);

/* Function Definitions */
static void ft_1(const emlrtStack *sp, const real_T ct[439], real_T D_dn3[49])
{
  emlrtStack st;
  real_T b_ct[315];
  real_T dv[49];
  real_T b_ct_tmp;
  real_T ct_tmp;
  real_T t1163;
  real_T t323;
  real_T t324;
  real_T t325;
  real_T t412;
  real_T t413;
  real_T t415;
  real_T t416;
  real_T t417;
  real_T t418;
  real_T t545;
  real_T t546;
  real_T t547;
  real_T t557;
  real_T t558;
  real_T t559;
  real_T t583;
  real_T t584;
  real_T t585;
  real_T t611;
  real_T t625;
  real_T t634;
  real_T t638;
  real_T t656;
  real_T t670;
  real_T t689;
  real_T t690;
  real_T t691;
  real_T t701;
  real_T t702;
  real_T t704;
  real_T t717;
  real_T t718;
  real_T t722;
  real_T t723;
  real_T t731;
  real_T t733;
  real_T t740;
  real_T t742;
  real_T t743;
  real_T t744;
  real_T t750;
  real_T t751;
  real_T t752;
  real_T t756;
  real_T t764;
  real_T t767;
  real_T t769;
  real_T t782;
  real_T t785;
  real_T t787;
  real_T t790;
  real_T t792;
  real_T t797;
  real_T t799;
  real_T t801;
  real_T t821;
  real_T t835;
  real_T t841;
  real_T t843;
  real_T t855;
  real_T t856;
  real_T t858;
  real_T t866;
  real_T t905;
  real_T t906;
  real_T t917;
  real_T t930;
  real_T t931;
  real_T t987;
  real_T t992;
  real_T t993;
  real_T t998;
  real_T t999;
  int32_T i;
  int32_T i1;
  st.prev = sp;
  st.tls = sp->tls;
  t670 = ct[379] * ct[430];
  t718 = (ct[189] + ct[312]) + ct[437];
  t992 = (((ct[13] + ct[404]) + ct[62]) + ct[95]) + ct[121];
  t993 = (((ct[22] + ct[410]) + ct[64]) + ct[94]) + ct[120];
  t323 = ct[15] + ct[96];
  t324 = ct[7] + ct[149];
  t325 = ct[12] + ct[130];
  t412 = ct[115] * ct[325];
  t413 = ct[116] * ct[389];
  t415 = ct[8] * ct[115];
  t416 = ct[18] * ct[115];
  t417 = ct[18] * ct[116];
  t418 = ct[29] * ct[117];
  t583 = ct[66] + ct[284];
  t584 = ct[67] + ct[256];
  t585 = ct[68] + ct[370];
  t611 = ct[18] * ct[337];
  t625 = ct[5] * 0.0607;
  t634 = ct[18] * ct[365];
  t638 = ct[293] * ct[389];
  t701 = ct[8] * ct[382];
  t702 = ct[8] * ct[383];
  t723 = ct[118] * ct[389] * 0.081;
  t731 = ct[56] + ct[141];
  t733 = ct[51] + ct[167];
  t740 = (ct[366] + ct[433]) + ct[19];
  t742 = (ct[196] + ct[291]) + ct[65];
  t744 = (ct[2] + ct[375]) + ct[17];
  t767 = ct[389] * t718;
  t769 = ct[18] * t718;
  t785 = ct[128] + ct[219];
  t787 = ct[144] + ct[240];
  t799 = ct[101] + ct[272];
  t835 = ct[188] + ct[341];
  t917 = ((((ct[170] + ct[411]) + ct[423]) + ct[26]) + ct[31]) + 17.0;
  t987 = (ct[287] + ct[346]) + ct[395];
  t998 = muDoubleScalarAbs(t992);
  t999 = muDoubleScalarAbs(t993);
  t545 = ct[325] * t324;
  t546 = ct[389] * t323;
  t547 = ct[413] * t325;
  t557 = ct[8] * t324;
  t558 = ct[18] * t323;
  t559 = ct[18] * t324;
  t656 = t417 * 607.0;
  t689 = ct[389] * t583;
  t690 = ct[389] * t584;
  t691 = ct[389] * t585;
  t704 = t417 * 0.126;
  t717 = ct[189] + t412;
  t722 = t413 * 0.081;
  t743 = (ct[206] + ct[344]) + -ct[438];
  t750 = ct[61] + -ct[127];
  t751 = ct[59] + -ct[147];
  t752 = ct[60] + -ct[146];
  t756 = ct[389] * t418 * 0.081;
  t764 = ct[18] * t325 * 0.081;
  t782 = ct[8] * t731;
  t790 = ct[389] * t740;
  t792 = ct[18] * t740;
  t797 = ct[29] * t742;
  t801 = ct[29] * t744;
  t821 = ct[29] * t799;
  t841 = ct[183] + -ct[358];
  t843 = ct[190] + -ct[350];
  t855 = ct[389] * t835;
  t856 = ct[18] * t835;
  t858 = ct[18] * (ct[192] + ct[337]);
  t866 = ct[1] * ct[159] * t733 * 0.42;
  t905 = (ct[107] + ct[151]) + -ct[209];
  t906 = (ct[171] + -ct[169]) + ct[233];
  t930 = muDoubleScalarAbs(t917);
  t931 = muDoubleScalarSign(t917);
  t835 = (ct[359] + -ct[363]) + ct[391];
  t917 = ct[18] * t987;
  t1163 =
      ((((((ct[222] + ct[226]) + -ct[223]) + ct[310]) + ct[329]) + -ct[304]) +
       -ct[327]) +
      ct[122] * ct[207];
  b_ct[0] = ct[0];
  b_ct[1] = ct[1];
  ct_tmp = ct[43] + ct[425];
  b_ct[2] = (ct_tmp + ct[354]) + t634;
  b_ct[3] = (((ct[53] + ct[426]) + ct[76]) + -ct[83]) + ct[119];
  b_ct[4] = ct[389] * t987;
  b_ct[5] = t917;
  b_ct[6] = t998 * t998;
  b_ct[7] = t999 * t999;
  b_ct[8] = (((ct[88] + ct[424]) + ct[91]) + ct[100]) + -(ct[5] * 607.0);
  b_ct[9] = ct[389] * t835;
  b_ct[10] = ct[18] * t835;
  b_ct[11] = ct[18] * ((ct[288] + ct[345]) + -ct[392]);
  b_ct[12] = ct[18] * ((ct[362] + -ct[361]) + ct[396]);
  b_ct[13] = -t917;
  b_ct[14] = ((((ct[34] + ct[217]) + ct[35]) + ct[38]) + ct[74]) + ct[85];
  b_ct[15] = ((ct[24] + ct[257]) + ct[282]) + -ct[324];
  b_ct[16] = ((ct[92] + ct[145]) + ct[354]) + t634;
  b_ct_tmp = ct[6] * ct[29];
  b_ct[17] = ((ct[180] + ct[205]) + ct[207]) + b_ct_tmp * 0.045;
  b_ct[18] = ((ct[71] + ct[125]) + ct[176]) + ct[18] * t583 * 607.0;
  b_ct[19] = (ct[54] + t412 * 1900.0) + ct[18] * t744 * 607.0;
  b_ct[20] = ((ct[82] + ct[134]) + ct[177]) + ct[18] * t585 * 607.0;
  t583 = ct[366] - t545;
  t585 = ct[18] * t583;
  b_ct[21] = -ct[29] * (t722 + t585 * 0.081);
  b_ct[22] = (-(ct[8] * ct[116] * 1900.0) + ct[389] * t324 * 607.0) +
             ct[18] * ct[293] * 607.0;
  t917 = ct[29] * ct[438];
  b_ct[23] = (((ct[129] + ct[229]) + ct[230]) + ct[267]) + -(t917 * 0.045);
  t634 = ct[413] * ct[438];
  b_ct[24] = (((ct[148] + ct[246]) + -ct[208]) + ct[280]) + t634 * 0.045;
  t987 = ct[29] * ct[434];
  b_ct[25] = (((ct[150] + ct[251]) + -ct[204]) + ct[279]) + t987 * 0.045;
  b_ct[26] = (((ct[155] + ct[255]) + -ct[214]) + ct[283]) + t634 * 0.126;
  b_ct[27] = (((ct[58] + -ct[143]) + ct[290]) + ct[352]) + t611;
  t634 = (ct[182] + ct[419]) + -ct[42];
  b_ct[28] = ((t634 + ct[290]) + ct[352]) + t611;
  b_ct[29] = (((ct[50] + ct[165]) + ct[369]) + ct[380]) + -t625;
  b_ct[30] = (((ct_tmp + ct[181]) + ct[212]) + ct[213]) + b_ct_tmp * 0.126;
  b_ct[31] = (((ct[194] + ct[303]) + ct[309]) + ct[326]) + -ct[387];
  b_ct[32] = (((ct[227] + ct[307]) + ct[328]) + -ct[330]) + ct[386];
  b_ct[33] = (((ct[200] + ct[315]) + ct[230] * ct[422]) + ct[333]) + -ct[390];
  b_ct[34] = (((ct[277] + ct[318]) + ct[321]) + ct[334]) + ct[388];
  t835 = ct[78] + ct[178];
  ct_tmp = (t835 + ct[415]) + ct[47];
  b_ct[35] = (((ct_tmp + ct[369]) + ct[380]) + -t625) + 0.34;
  b_ct[36] = ct[8];
  b_ct[37] = (((ct[126] + ct[216]) + ct[18] * ct[343]) + ct[405]) +
             ct[18] * ct[436] * -0.0607;
  b_ct[38] = (((ct[136] + ct[239]) + ct[18] * ct[360]) + t670) +
             ct[4] * ct[18] * -0.0607;
  b_ct[39] = (((ct[348] + ct[346] * ct[389]) + ct[287] * ct[389]) + ct[409]) +
             ct[188] * ct[274];
  b_ct[40] = (((ct[364] + ct[18] * ct[341]) + ct[363] * ct[389]) + ct[407]) +
             ct[96] * ct[274] * -0.0607;
  b_ct[41] = ((((ct[202] + ct[242]) + ct[299]) + ct[243] * ct[389]) + ct[374]) +
             ct[122] * ct[230];
  b_ct[42] = ct[10];
  b_ct[43] = ((((ct[102] + ct[140]) + ct[198]) + ct[18] * ct[362]) +
              ct[18] * ct[373] * -0.0607) +
             ct[188] * ct[401];
  b_ct[44] = ((((ct[201] + ct[245]) + ct[317]) + -ct[300]) + -ct[314]) +
             ct[149] * ct[207];
  b_ct[45] =
      ((((t634 + ct[156]) + ct[260]) + -ct[211]) + ct[281]) + t987 * 0.126;
  b_ct[46] = ((((ct[221] + -ct[225]) + ct[297]) + ct[371]) + ct[376]) +
             ct[96] * ct[230];
  b_ct[47] = ((((ct[157] + -ct[163]) + ct[193]) + ct[406]) +
              ct[18] * ct[196] * -0.0607) +
             ct[183] * ct[401];
  b_ct[48] =
      ((((ct[224] + ct[266]) + ct[302]) + -ct[305]) + -ct[311]) + ct[402];
  b_ct[49] = ((((-ct[138] + ct[237]) + ct[349]) + ct[342] * ct[389]) +
              ct[18] * ct[336]) +
             ct[3] * ct[18] * -0.0607;
  b_ct[50] = ((((ct[162] + ct[215]) + ct[379]) + ct[335] * ct[389]) + ct[408]) +
             ct[18] * ct[435] * -0.0607;
  b_ct[51] = (((((ct_tmp + ct[132]) + ct[234]) + ct[235]) + ct[269]) +
              -(t917 * 0.126)) +
             0.34;
  b_ct[52] = (ct[27] + ct[63]) * t930 * t931 * 2.0;
  b_ct[53] =
      ((((((ct[203] + ct[243]) + -ct[241]) + ct[202] * ct[389]) + ct[313]) +
        ct[242] * ct[389]) +
       -ct[319]) +
      ct[385];
  b_ct[54] = t1163;
  b_ct[55] =
      ((((((ct[220] + ct[228]) + ct[265]) + ct[308]) + ct[225] * ct[389]) +
        -ct[301]) +
       ct[372]) +
      -ct[381];
  b_ct[56] =
      ((((((ct[199] + ct[244]) + ct[278]) + ct[320]) + -ct[298]) + -ct[316]) +
       -ct[322]) +
      ct[96] * ct[207];
  b_ct[57] = t1163 * t1163;
  b_ct[58] =
      ((((((ct[103] + ct[135]) + -ct[197]) + ct[351]) + ct[341] * ct[389]) +
        ct[18] * ct[359]) +
       ct[18] * ct[384] * -0.0607) +
      ct[18] * ct[391];
  b_ct[59] =
      ((((((ct[158] + -ct[160]) + -ct[218]) + ct[377]) + ct[358] * ct[389]) +
        ct[18] * ct[346]) +
       ct[18] * ct[287]) +
      ct[18] * ct[395];
  b_ct[60] = ct[18];
  ct_tmp = t998 * muDoubleScalarSign(t992);
  b_ct[61] =
      ct_tmp * ((((ct[32] + ct[48]) + ct[97]) + ct[112]) + ct[123]) * 2.0;
  b_ct_tmp = t999 * muDoubleScalarSign(t993);
  b_ct[62] =
      b_ct_tmp * ((((ct[28] + ct[44]) + ct[93]) + ct[114]) + ct[124]) * 2.0;
  b_ct[63] = ct_tmp *
             ((((((ct[428] + ct[429]) + ct[30]) + ct[36]) + ct[75]) + ct[111]) +
              ct[175]) *
             2.0;
  b_ct[64] = b_ct_tmp *
             ((((((ct[431] + ct[432]) + ct[33]) + ct[39]) + ct[86]) + ct[110]) +
              ct[174]) *
             2.0;
  b_ct[65] = ct[23];
  b_ct[66] = ct[24];
  b_ct[67] = ct[29];
  b_ct[68] = ct[37];
  b_ct[69] = ct[40];
  b_ct[70] = ct[41];
  b_ct[71] = ct[43];
  b_ct[72] = ct[45];
  b_ct[73] = ct[46];
  b_ct[74] = ct[47];
  b_ct[75] = ct[69];
  b_ct[76] = -ct[42];
  b_ct[77] = -ct[43];
  b_ct[78] = ct[78];
  b_ct[79] = ct[81];
  b_ct[80] = -ct[45];
  b_ct[81] = -ct[46];
  b_ct[82] = ct[90];
  b_ct[83] = ct[98];
  b_ct[84] = ct[99];
  b_ct[85] = -ct[89];
  b_ct[86] = ct[142];
  b_ct[87] = ct[159];
  b_ct[88] = ct[164];
  b_ct[89] = ct[166];
  b_ct[90] = ct[178];
  b_ct[91] = ct[182];
  b_ct[92] = -ct[164];
  b_ct[93] = ct[254];
  b_ct[94] = ct[261];
  b_ct[95] = ct[268];
  b_ct[96] = ct[270];
  b_ct[97] = ct[273];
  b_ct[98] = ct[276];
  b_ct[99] = ct[117] * ct[413];
  b_ct[100] = t415;
  b_ct[101] = t418;
  b_ct[102] = -ct[270];
  b_ct[103] = ct[289];
  b_ct[104] = ct[325];
  b_ct[105] = ct[339];
  b_ct[106] = ct[340];
  b_ct[107] = ct[345];
  b_ct[108] = ct[352];
  b_ct[109] = ct[353];
  b_ct[110] = ct[357];
  b_ct[111] = ct[359];
  b_ct[112] = ct[366];
  b_ct[113] = ct[368];
  b_ct[114] = t545;
  b_ct[115] = t546;
  b_ct[116] = t547;
  b_ct[117] = ct[373];
  b_ct[118] = -t418;
  b_ct[119] = ct[378];
  b_ct[120] = -ct[359];
  b_ct[121] = ct[389];
  b_ct[122] = ct[349] * ct[430];
  b_ct[123] = ct[367] * ct[430];
  b_ct[124] = ct[400];
  b_ct[125] = t638;
  b_ct[126] = ct[292] * ct[413];
  b_ct[127] = ct[29] * (ct[6] + ct[274]);
  b_ct[128] = ct[294] * ct[413];
  b_ct[129] = ct[295] * ct[413];
  b_ct[130] = ct[29] * ct[296];
  b_ct[131] = -t546;
  b_ct[132] = -t547;
  b_ct[133] = ct[115] * ct[389] * 607.0;
  b_ct[134] = t413 * 607.0;
  b_ct[135] = -t559;
  b_ct[136] = t656;
  b_ct[137] = ct[405];
  b_ct[138] = t670;
  b_ct[139] = -t656;
  b_ct[140] = -(t415 * 1900.0);
  b_ct[141] = ct[412];
  b_ct[142] = ct[413] * (ct[72] + ct[306]);
  b_ct[143] = ct[325] * ct[382];
  b_ct[144] = t413 * 0.126;
  b_ct[145] = t546 * 607.0;
  b_ct[146] = t545 * 1900.0;
  b_ct[147] = ct[413];
  b_ct[148] = t701;
  b_ct[149] = t702;
  b_ct[150] = t704;
  b_ct[151] = t558 * 607.0;
  b_ct[152] = t557 * 1900.0;
  b_ct[153] = ct[415];
  ct_tmp = ct[238] * 0.4 - ct[421];
  b_ct[154] = -ct[325] * ct_tmp;
  b_ct_tmp = ct[355] * 0.4 - ct[414];
  b_ct[155] = -ct[8] * b_ct_tmp;
  b_ct[156] = -ct[8] * ct_tmp;
  b_ct[157] = -(t416 * 0.126);
  b_ct[158] = -t704;
  b_ct[159] = -(ct[8] * t323 * 1900.0);
  b_ct[160] = ct[416];
  b_ct[161] = t546 * 0.126;
  b_ct[162] = t722;
  b_ct[163] = t723;
  b_ct[164] = -t701;
  b_ct[165] = -t702;
  b_ct[166] = t558 * 0.126;
  b_ct[167] = t559 * 0.126;
  b_ct[168] = t416 * 0.081;
  b_ct[169] = ct[417];
  b_ct[170] = t417 * 0.081;
  b_ct[171] = ct[57] + ct[137];
  b_ct[172] = t733;
  b_ct[173] = ct[325] * ct_tmp;
  b_ct[174] = -t722;
  b_ct[175] = ct[8] * b_ct_tmp;
  b_ct[176] = (ct[5] + ct[399]) + ct[21];
  b_ct[177] = t638 * 0.126;
  b_ct[178] = ct[18] * t584 * 607.0;
  b_ct[179] = ct[49] + -ct[161];
  b_ct[180] = t751;
  b_ct[181] = t752;
  ct_tmp = ct[8] * ct[413];
  b_ct[182] = ct_tmp * ct[116] * 0.081;
  b_ct[183] = t756;
  b_ct[184] = t545 * 0.081;
  b_ct[185] = t546 * 0.081;
  b_ct[186] = ct[418];
  b_ct[187] = t557 * 0.081;
  b_ct[188] = t558 * 0.081;
  b_ct[189] = t559 * 0.081;
  b_ct[190] = t764;
  b_ct[191] = ct[291] + t557;
  b_ct[192] = ct[413] * t718;
  b_ct[193] = t769;
  b_ct[194] = ct[419];
  b_ct[195] = ct[325] * t413 * -0.081;
  b_ct[196] = -t756;
  b_ct[197] = t733 * t733;
  b_ct[198] = t689 * 0.126;
  b_ct[199] = t690 * 0.126;
  b_ct[200] = t691 * 0.126;
  b_ct[201] = ct_tmp * t323 * 0.081;
  b_ct[202] = ct[325] * t417 * 0.0607;
  b_ct[203] = ct[420];
  b_ct[204] = t545 * 0.0607;
  b_ct[205] = t557 * 0.0607;
  b_ct[206] = t785;
  b_ct[207] = t787;
  b_ct[208] = ct[413] * t740;
  b_ct[209] = ct[413] * t742;
  b_ct[210] = ct[413] * t743;
  b_ct[211] = ct[413] * t744;
  b_ct[212] = t797;
  b_ct[213] = ct[29] * t743;
  b_ct[214] = ct[325] * t546 * -0.081;
  b_ct[215] = t801;
  b_ct[216] = ct[325] * t750;
  b_ct[217] = ct[8] * t750;
  b_ct[218] = t689 * 0.081;
  b_ct[219] = t691 * 0.081;
  b_ct[220] = ct[325] * t558 * 0.0607;
  b_ct[221] = -t782;
  b_ct[222] = ct[389] * t797;
  b_ct[223] = ct[389] * t801;
  b_ct[224] = t767 * 607.0;
  b_ct[225] = t769 * 607.0;
  b_ct[226] = ct[413] * (ct[131] + ct[232]);
  b_ct[227] = ct[413] * (ct[152] + ct[253]);
  b_ct[228] = ct[413] * (ct[172] + ct[231]);
  b_ct[229] = ct[9] + t689;
  b_ct[230] = ct[413] * t799;
  b_ct[231] = ct[425];
  b_ct[232] = ct[182] + t752;
  b_ct[233] = t821;
  b_ct[234] = ct[14] + t691;
  b_ct[235] = t790 * 607.0;
  b_ct[236] = t767 * 0.126;
  b_ct[237] = t792 * 607.0;
  b_ct[238] = t769 * 0.126;
  b_ct[239] = ct[18] * t742 * 607.0;
  b_ct[240] = ct[413] * (ct[153] + -ct[252]);
  b_ct[241] = ct[413] * (ct[108] + -ct[264]);
  b_ct[242] = ct[413] * (ct[109] + -ct[263]);
  b_ct[243] = -t821;
  b_ct[244] = ct[389] * t717 * 0.081;
  b_ct[245] = t790 * 0.126;
  b_ct[246] = ct[18] * t717 * 0.081;
  b_ct[247] = t792 * 0.126;
  b_ct[248] = ct[427];
  b_ct[249] = ct[413] * (ct[236] + ct[331]);
  b_ct[250] = ct[413] * (ct[262] + ct[338]);
  b_ct[251] = ct[389] * (ct[187] + ct[342]);
  b_ct[252] = t855;
  b_ct[253] = t856;
  b_ct[254] = ct[18] * (ct[184] + ct[365]);
  b_ct[255] = t858;
  b_ct[256] = ct[412] + t785;
  b_ct[257] = (ct[52] + ct[80]) + -(ct[438] * 607.0);
  b_ct[258] = (ct[55] + ct[77]) + ct[179];
  b_ct[259] = ct[417] + t787;
  b_ct[260] = t866;
  b_ct[261] = ct[389] * t841;
  b_ct[262] = ct[389] * (ct[191] + -ct[335]);
  b_ct[263] = ct[389] * t843;
  b_ct[264] = ct[18] * t841;
  b_ct[265] = ct[18] * t843;
  b_ct[266] = -t866;
  b_ct[267] = -(ct[29] * (ct[275] + ct[332]));
  b_ct[268] = -t855;
  b_ct[269] = -t856;
  b_ct[270] = -t858;
  b_ct[271] = ct[29] * ct[389] * (ct[373] - t415) * -0.081;
  b_ct[272] = ct[389] * t583 * -0.081;
  b_ct[273] = t585 * -0.081;
  b_ct[274] = ct[185] + t412 * 0.081;
  b_ct[275] = ct[18] * (ct[347] + ct[393]);
  b_ct[276] = (t835 + t733) + 0.34;
  b_ct[277] = ct[18] * (ct[343] + -ct[394]);
  b_ct[278] = ct[18] * (ct[336] + -ct[397]);
  b_ct[279] = ct[18] * (ct[360] + -ct[398]);
  b_ct[280] = ct[104] + ct[325] * t731;
  b_ct[281] = ct[186] + ct[389] * t325 * 0.081;
  b_ct[282] = -ct[166] + t782;
  b_ct[283] = ct[287] + t412 * 0.0607;
  ct_tmp = ct[271] - t415 * 0.081;
  b_ct[284] = -ct[413] * ct_tmp;
  b_ct[285] = -ct[29] * (ct[273] - t723);
  b_ct[286] = -ct[29] * ct_tmp;
  b_ct[287] = -ct[361] + t415 * 0.0607;
  b_ct[288] = ct[413] * ((ct[106] + ct[154]) + ct[210]);
  b_ct[289] = t930;
  b_ct[290] = t931;
  b_ct[291] = ((ct[25] + ct[79]) + ct[84]) + ct[133];
  b_ct[292] = ct[413] * ct_tmp;
  b_ct[293] = ct[413] * ((ct[168] + -ct[173]) + ct[195]);
  b_ct[294] = t413 + t792;
  b_ct[295] = -ct[29] * (ct[276] - t764);
  b_ct[296] = ct[413] * t905;
  b_ct[297] = t558 + t767;
  b_ct[298] = ct[413] * t906;
  b_ct[299] = ct[29] * t905;
  b_ct[300] = ct[29] * t906;
  b_ct[301] = ct[29] * (t559 - t638);
  b_ct[302] = ct[29] * (t416 - t690);
  b_ct[303] = t930 * t930;
  b_ct[304] = (ct[24] + ct[353]) + -ct[400];
  b_ct[305] =
      ct[413] * (ct[11] + ct[139]) + ct[29] * ((ct[403] + ct[434]) + ct[20]);
  b_ct[306] = (ct[248] + -ct[247]) + ct[323];
  ct_tmp = ct[5] * ct[29];
  b_ct[307] = (ct[249] + -ct[250]) + ct_tmp * 0.045;
  b_ct[308] = (ct[258] + -ct[259]) + ct_tmp * 0.126;
  b_ct[309] = (ct[105] + ct[353]) + -ct[400];
  ct_tmp = t417 - t790;
  b_ct[310] = -ct[29] * ct_tmp;
  b_ct[311] = t733 * t751 * 2.0;
  b_ct[312] = t733 * t752 * 2.0;
  b_ct[313] = ct[413] * ct_tmp;
  b_ct[314] = (ct[289] + ct[356]) + -(ct[438] * 0.0607);
  st.site = &gc_emlrtRSI;
  ft_2(&st, b_ct, dv);
  for (i = 0; i < 7; i++) {
    for (i1 = 0; i1 < 7; i1++) {
      D_dn3[i1 + 7 * i] = dv[i + 7 * i1];
    }
  }
}

static void ft_2(const emlrtStack *sp, const real_T ct[315], real_T D_dn3[49])
{
  emlrtStack st;
  real_T a_tmp;
  real_T b_a_tmp;
  real_T b_t1064_tmp;
  real_T b_t1273_tmp;
  real_T b_t1281_tmp;
  real_T b_t1284_tmp;
  real_T b_t1285_tmp;
  real_T b_t1287_tmp;
  real_T b_t1289_tmp;
  real_T b_t1302_tmp;
  real_T b_t1304_tmp;
  real_T b_t1351_tmp;
  real_T b_t1353_tmp;
  real_T b_t1357_tmp;
  real_T b_t1361_tmp;
  real_T b_t1369_tmp;
  real_T c_a_tmp;
  real_T c_t1353_tmp;
  real_T c_t1357_tmp;
  real_T d;
  real_T d1;
  real_T d2;
  real_T d3;
  real_T d4;
  real_T d5;
  real_T d6;
  real_T d_t1353_tmp;
  real_T d_t1357_tmp;
  real_T t1005;
  real_T t1016;
  real_T t1017;
  real_T t1022;
  real_T t1031;
  real_T t1035;
  real_T t1037;
  real_T t1044;
  real_T t1051;
  real_T t1052;
  real_T t1052_tmp;
  real_T t1053;
  real_T t1056;
  real_T t1064;
  real_T t1064_tmp;
  real_T t1066;
  real_T t1069;
  real_T t1083;
  real_T t1084;
  real_T t1087;
  real_T t1088;
  real_T t1091;
  real_T t1095;
  real_T t1097;
  real_T t1099;
  real_T t1103;
  real_T t1104;
  real_T t1106;
  real_T t1108;
  real_T t1112;
  real_T t1118;
  real_T t1121;
  real_T t1122;
  real_T t1125;
  real_T t1129;
  real_T t1133;
  real_T t1134;
  real_T t1135;
  real_T t1136;
  real_T t1138;
  real_T t1139;
  real_T t1140;
  real_T t1141;
  real_T t1142;
  real_T t1145;
  real_T t1146;
  real_T t1147;
  real_T t1148;
  real_T t1149;
  real_T t1153;
  real_T t1153_tmp;
  real_T t1154;
  real_T t1155;
  real_T t1156_tmp;
  real_T t1161;
  real_T t1164;
  real_T t1166;
  real_T t1166_tmp;
  real_T t1171;
  real_T t1172;
  real_T t1174;
  real_T t1176;
  real_T t1176_tmp;
  real_T t1177;
  real_T t1179;
  real_T t1180;
  real_T t1181;
  real_T t1182;
  real_T t1184;
  real_T t1187;
  real_T t1188;
  real_T t1189;
  real_T t1191;
  real_T t1192;
  real_T t1193;
  real_T t1194;
  real_T t1195;
  real_T t1196;
  real_T t1198;
  real_T t1201;
  real_T t1202;
  real_T t1203;
  real_T t1204;
  real_T t1208;
  real_T t1210;
  real_T t1211;
  real_T t1213;
  real_T t1216;
  real_T t1218;
  real_T t1219;
  real_T t1230;
  real_T t1233;
  real_T t1235_tmp;
  real_T t1235_tmp_tmp;
  real_T t1237;
  real_T t1238;
  real_T t1242_tmp;
  real_T t1245;
  real_T t1247_tmp;
  real_T t1250;
  real_T t1251;
  real_T t1258;
  real_T t1260_tmp;
  real_T t1262_tmp;
  real_T t1264;
  real_T t1268_tmp;
  real_T t1270_tmp;
  real_T t1273;
  real_T t1273_tmp;
  real_T t1278_tmp;
  real_T t1280;
  real_T t1280_tmp;
  real_T t1280_tmp_tmp;
  real_T t1281;
  real_T t1281_tmp;
  real_T t1284;
  real_T t1284_tmp;
  real_T t1285;
  real_T t1285_tmp;
  real_T t1287;
  real_T t1287_tmp;
  real_T t1288;
  real_T t1288_tmp;
  real_T t1289;
  real_T t1289_tmp;
  real_T t1290;
  real_T t1290_tmp;
  real_T t1300;
  real_T t1301;
  real_T t1302;
  real_T t1302_tmp;
  real_T t1303;
  real_T t1303_tmp;
  real_T t1304;
  real_T t1304_tmp;
  real_T t1305;
  real_T t1305_tmp;
  real_T t1312;
  real_T t1320_tmp;
  real_T t1324;
  real_T t1325;
  real_T t1328;
  real_T t1335;
  real_T t1337;
  real_T t1342;
  real_T t1342_tmp;
  real_T t1348;
  real_T t1348_tmp_tmp;
  real_T t1351;
  real_T t1351_tmp;
  real_T t1351_tmp_tmp;
  real_T t1352;
  real_T t1352_tmp;
  real_T t1353;
  real_T t1353_tmp;
  real_T t1354;
  real_T t1357;
  real_T t1357_tmp;
  real_T t1358;
  real_T t1361;
  real_T t1361_tmp;
  real_T t1362;
  real_T t1362_tmp;
  real_T t1363;
  real_T t1365;
  real_T t1369;
  real_T t1369_tmp;
  real_T t1370;
  real_T t1372;
  real_T t1372_tmp;
  real_T t1375;
  real_T t1378;
  real_T t1379;
  real_T t1380;
  real_T t1383;
  real_T t1384;
  real_T t1387;
  real_T t1389;
  real_T t1392;
  real_T t1394;
  real_T t1396;
  real_T t881;
  real_T t881_tmp;
  real_T t901;
  real_T t907;
  real_T t910;
  real_T t923;
  real_T t926;
  real_T t928;
  real_T t929;
  real_T t940;
  real_T t962;
  real_T t972;
  real_T t978;
  real_T t979;
  real_T t980;
  real_T t986;
  st.prev = sp;
  st.tls = sp->tls;
  t1172 = ct[56] * ct[56];
  t1179 = ct[58] * ct[58];
  t1180 = ct[59] * ct[59];
  t881_tmp = ct[67] * ct[121];
  t881 = t881_tmp * ct[191] * 0.081;
  t901 = ct[94] + ct[187];
  t907 = ct[147] * ct[274];
  t910 = ct[86] + ct[217];
  t926 = ct[67] * ct[281];
  a_tmp = ct[89] + ct[221];
  t929 = a_tmp * a_tmp;
  t940 = ct[99] + ct[213];
  t980 = (ct[89] + ct[203]) + ct[221];
  t1383 = ct[0] + ct[248];
  t986 = t1383 + ct[282];
  t1022 = ct[166] + ct[236];
  t1031 = ct[8] * ct[8];
  t1037 = ct[188] + ct[244];
  t1044 = ct[170] + ct[272];
  t1052_tmp = ct[185] - ct[246];
  t1052 = -ct[67] * t1052_tmp;
  t1053 = (ct[66] + ct[106]) + ct[267];
  t1083 = ct[209] + ct[310];
  t1091 = ((ct[79] + ct[134]) + ct[152]) + ct[237];
  t1328 = ct[65] + ct[143];
  t1103 = t1328 + ct[12];
  t1325 = (ct[68] + ct[141]) + ct[83];
  t1104 = (t1325 + ct[137]) + ct[277];
  t1312 = (ct[70] + ct[169]) + ct[84];
  t1106 = (t1312 + ct[138]) + ct[279];
  t1108 = ct[4] + ct[264];
  t1112 = (ct[196] + ct[241]) + ct[254];
  t1118 = (ct[103] + ct[263]) + ct[295];
  t1121 = (ct[165] + ct[262]) + ct[275];
  t1122 = (((ct[91] + ct[194]) + ct[76]) + ct[242]) + ct[285];
  t1125 = (ct[175] + ct[251]) + ct[278];
  t1147 = ct[3] * ct[8] * 2.0;
  t1166_tmp = (ct[69] + ct[160]) + ct[156];
  t1166 = (t1166_tmp + ct[252]) + ct[10];
  t1176_tmp = (t1383 + ct[81]) + ct[148];
  t1176 = (t1176_tmp + ct[261]) + ct[13];
  t923 = ct[147] * t901;
  t928 = t910 * t910;
  t962 = t940 * t940;
  t972 = ct[160] + t910;
  t978 = ct[67] * (ct[105] + ct[218]);
  t979 = ct[67] * (ct[110] + ct[219]);
  t1016 = ct[67] * (ct[168] + ct[214]);
  t1017 = ct[67] * (ct[189] + ct[195]);
  t1035 = ct[67] * t1022;
  t1051 = ct[67] * t1037;
  t1056 = ct[67] * t1044;
  t1064_tmp = ct[36] * ct[171];
  b_t1064_tmp = t1064_tmp * t910;
  t1064 = b_t1064_tmp * 2.0;
  t1066 = ct[206] * t910 * 2.0;
  t1069 = ct[67] * ct[176] * t940 * 2.0;
  t1087 = t1083 * t1083;
  t1095 = ((ct[82] + ct[140]) + -ct[145]) + ct[225];
  t1097 = t1091 * t1091;
  t1383 = ct[150] - ct[245];
  t1139 = ct[299] + ct[147] * t1383;
  t1140 = (((((ct[78] + ct[90]) + ct[153]) + ct[74]) + ct[230]) + t926) + 0.34;
  t1145 = (t1328 + ct[271]) + t907;
  t1146 = (t1328 + ct[223] * 0.126) + ct[288];
  t1156_tmp = ct[67] * ct[294] * t1083;
  t1161 = ((ct[113] + ct[230]) + -ct[265]) + t926;
  t1328 = (ct[73] + ct[203]) + ct[164];
  t1171 = (t1328 + -ct[261]) + ct[5];
  t1177 = ct[75] + t1166;
  t1191 = (t1166_tmp + ct[296]) + -ct[67] * t1383;
  t1192 = (ct[264] + ct[121] * ct[283]) + t1052;
  t1210 = ((ct[85] + ct[146]) + ct[239]) * t1091 * 2.0;
  t1216 = t1083 * (ct[212] + ct[313]) * 2.0;
  d = (ct[6] + ct[303]) + ct[7];
  st.site = &qc_emlrtRSI;
  if (d < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &st, &h_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
  }
  d = muDoubleScalarSqrt(d);
  t926 = 1.0 / d;
  t1005 = ct[75] + t972;
  t1084 = ct[211] + -(ct[67] * ct[297]);
  t1099 = t1095 * t1095;
  t1129 = (ct[149] + ct[182]) + t1017;
  t1133 = ct[300] + ct[147] * t1022;
  t1134 = (ct[155] + ct[201]) + t1016;
  t1135 = ct[286] + ct[147] * t1037;
  t1136 = (t1325 + ct[226]) + ct[67] * (ct[93] + ct[198]);
  t1138 = (t1312 + ct[227]) + ct[67] * (ct[95] + ct[200]);
  t1142 = ct[67] * t901 + ct[147] * t1044;
  t1148 = (t1325 + ct[249]) + t978;
  t1149 = (t1312 + ct[250]) + t979;
  t1153_tmp = ct[72] + ct[173];
  t1153 = (t1153_tmp + -(ct[222] * 0.126)) + ct[293];
  t1164 = t1161 * t1161;
  t1174 = (ct[126] + ct[67] * ct[229]) * t1083 * 2.0;
  t1182 = (ct[151] + ct[224]) * t1095 * 2.0;
  t1184 = (ct[208] + -ct[222]) * t1083 * 2.0;
  t1187 = ((ct[182] + ct[202]) + ct[262]) + t1017;
  t1188 = ((ct[123] + ct[250]) + -ct[279]) + t979;
  t1189 = ((ct[122] + ct[249]) + -ct[277]) + t978;
  t1194 = (t1328 + -ct[298]) + t1035;
  t1195 = ((ct[201] + ct[220]) + ct[251]) + t1016;
  t1201 = ct[75] + t1191;
  t1202 = ct[19] * t1095 * 2.0;
  t1203 = ct[20] * t1095 * 2.0;
  t1204 = (t1328 + ct[292]) + t1051;
  t1211 = ((ct[133] + ct[159]) + ct[178]) * t1095 * 2.0;
  b_a_tmp = ((ct[261] + ct[292]) - ct[60] * ct[283]) + t1051;
  t1230 = b_a_tmp * b_a_tmp;
  t1383 = ct[111] - ct[204];
  c_a_tmp = ((ct[252] - t923) + t1056) + ct[60] * t1383;
  t1233 = c_a_tmp * c_a_tmp;
  t1235_tmp_tmp = ct[233] - ct[147] * ct[281];
  t1235_tmp = t1161 * t1235_tmp_tmp;
  t1245 = t1112 * t1161 * 2.0;
  t1247_tmp = t1118 * t1161;
  t1273_tmp = ct[147] * (ct[96] - ct[184]);
  b_t1273_tmp = (-t881 + ct[60] * (ct[107] + ct[205])) + t1273_tmp;
  t1273 = b_t1273_tmp * c_a_tmp * 2.0;
  t1280_tmp_tmp = ct[162] + ct[60] * (ct[112] - ct[114]) * 0.081;
  t1280_tmp = (ct[269] + ct[67] * t1280_tmp_tmp) + ct[121] * t1383;
  t1280 = t1280_tmp * c_a_tmp * 2.0;
  t1312 = t926 * ((ct[52] + ct[61]) + -ct[62]) / 100.0;
  t1325 =
      t926 * ((-(ct[289] * ct[290] * ct[14] * 2.0) + ct[63]) + ct[64]) / 100.0;
  t1088 = t1084 * t1084;
  t1154 = 1.0 / ((ct[197] + t928) + t929);
  t1181 = (ct[192] + ct[223]) * t1084 * 2.0;
  t1193 = (ct[142] + ct[302]) * t1084 * 2.0;
  t1198 = (t1176_tmp + ct[298]) + -t1035;
  t1208 = (t1166_tmp + t923) + -t1056;
  t1278_tmp = t1189 * c_a_tmp;
  t1285_tmp = ct[8] * ct[257];
  b_t1285_tmp = ct[136] - ct[235];
  t1285 = (t1285_tmp * 2.0 + t1182) + t1091 * b_t1285_tmp * -2.0;
  t1290_tmp = ct[18] * t1091;
  t1290 = (-t1147 + t1290_tmp * 2.0) + t1203;
  t1141 = (ct[149] + ct[228]) + -(ct[67] * (-ct[167] + ct[177]));
  t1155 = t1154 * t1154;
  t1213 = ct[75] + t1208;
  t1218 = t928 * t1154;
  t1219 = t929 * t1154;
  t1242_tmp = ct[172] * ct[276];
  t978 = t1242_tmp * t910;
  t1017 = t978 * t1154;
  t1250 = 1.0 / ((t1031 + t1097) + t1099);
  t1281_tmp = t940 * ct[305];
  b_t1281_tmp = (ct[128] + ct[67] * ct[234]) * t1084;
  t1281 = (t1281_tmp * 2.0 + b_t1281_tmp * 2.0) + -t1174;
  t1284_tmp = (ct[129] + ct[301]) * t1083;
  b_t1284_tmp = ct[42] - ct[130];
  t1284 = (t940 * b_t1284_tmp * -2.0 + t1284_tmp * 2.0) + t1193;
  t1287_tmp = ct[8] * ct[258];
  b_t1287_tmp = ct[22] * t1091;
  t1287 = (t1287_tmp * 2.0 + b_t1287_tmp * 2.0) + -t1211;
  t1288_tmp = ct[8] * ct[291];
  t1288 = (t1288_tmp * 2.0 + -t1202) + t1210;
  t1289_tmp = t940 * (ct[101] - ct[210]);
  b_t1289_tmp = (ct[215] + ct[147] * ct[297]) * t1084;
  t1289 = (t1289_tmp * 2.0 + b_t1289_tmp * 2.0) + t1216;
  t1300 = 1.0 / ((t1164 + t1230) + t1233);
  t1342_tmp =
      (-t907 + ct[60] * ct[287]) + t881_tmp * (ct[117] - ct[100]) * 0.081;
  t1342 = (-t1245 + t1342_tmp * b_a_tmp * 2.0) + t1273;
  t1196 = ct[197] * t1154 - 1.0;
  t1237 = 1.0 / ((t962 + t1087) + t1088);
  t1251 = t1250 * t1250;
  t1268_tmp = t1099 * t1250;
  t1301 = t1300 * t1300;
  t1304_tmp = ct[29] * ct[35];
  b_t1304_tmp = t1304_tmp * ct[58];
  t1304 = b_t1304_tmp * t1250 * 1.0E+8;
  t1305_tmp = t1304_tmp * ct[59];
  t1305 = t1305_tmp * t1250 * 1.0E+8;
  t1335 = t1230 * t1300;
  t1337 = t1233 * t1300;
  t1348_tmp_tmp = -t1140 * t1161;
  t1383 = t1348_tmp_tmp * t1300;
  t1348 = t1383 * c_a_tmp;
  t1351_tmp = ct[0] - t1204;
  t1351_tmp_tmp = t1300 * t1351_tmp;
  b_t1351_tmp = t1351_tmp_tmp * b_a_tmp;
  t1351 = b_t1351_tmp * c_a_tmp;
  t1352_tmp = t1213 * t1300;
  t1352 = t1352_tmp * b_a_tmp * c_a_tmp;
  t1238 = t1237 * t1237;
  t1260_tmp = t1088 * t1237;
  t1264 = t1031 * t1250 - 1.0;
  t1270_tmp = t1097 * t1250 - 1.0;
  t1302_tmp = ct[23] * ct[51];
  b_t1302_tmp = t1302_tmp * ct[54];
  t1302 = b_t1302_tmp * t1237 * 493.82716049382719;
  t1303_tmp = t1302_tmp * ct[56];
  t1303 = t1303_tmp * t1237 * 493.82716049382719;
  t1324 = t1164 * t1300 - 1.0;
  t1258 = t962 * t1237 - 1.0;
  t1262_tmp = t1087 * t1237 - 1.0;
  t1320_tmp = ct[172] * t910;
  t979 = t1320_tmp * t1005;
  t1022 = (ct[276] * t1196 + -ct[172] * t986 * t1154 * a_tmp) + t979 * t1154;
  t901 = (t1005 * (t1218 - 1.0) + t1017) + -t910 * t986 * t1154 * a_tmp;
  t1328 = (t986 * (t1219 - 1.0) + -ct[172] * ct[276] * t1154 * a_tmp) +
          -t910 * t1005 * t1154 * a_tmp;
  t1037 = muDoubleScalarAbs(t1022);
  t1044 = muDoubleScalarAbs(t901);
  t926 = muDoubleScalarAbs(t1328);
  t1357_tmp = ct[29] * ct[58];
  b_t1357_tmp = ct[29] * ct[59];
  c_t1357_tmp = t1357_tmp * t1177;
  d_t1357_tmp = b_t1357_tmp * t1176;
  t1357 = (ct[35] * t1264 + c_t1357_tmp * t1250 * 1.0E+8) +
          -(d_t1357_tmp * t1250 * 1.0E+8);
  t1369_tmp = ct[58] * ct[59];
  b_t1369_tmp = t1369_tmp * t1176;
  t1369 = (t1177 * t1270_tmp + t1304) + -(b_t1369_tmp * t1250 * 1.0E+8);
  t1379 = (t1140 * t1324 + -t1161 * t1300 * t1351_tmp * b_a_tmp) +
          -t1161 * t1213 * t1300 * c_a_tmp;
  t1383 = ((t1335 - 1.0) * t1351_tmp + t1383 * b_a_tmp) + t1352;
  t1387 = (t1213 * (t1337 - 1.0) + t1348) + t1351;
  t1353_tmp = ct[23] * ct[54];
  b_t1353_tmp = ct[23] * ct[56];
  c_t1353_tmp = t1353_tmp * t1198;
  d_t1353_tmp = b_t1353_tmp * t1201;
  t1353 = (ct[51] * t1258 + -(c_t1353_tmp * t1237 * 493.82716049382719)) +
          d_t1353_tmp * t1237 * 493.82716049382719;
  t1358 = muDoubleScalarAbs(t1357);
  t1361_tmp = ct[54] * ct[56];
  b_t1361_tmp = t1361_tmp * t1198;
  t1361 =
      (t1201 * t1262_tmp + t1303) + -(b_t1361_tmp * t1237 * 493.82716049382719);
  t1370 = muDoubleScalarAbs(t1369);
  t1372_tmp = t1369_tmp * t1177;
  t1372 = (-(t1176 * (t1268_tmp - 1.0)) + t1305) + t1372_tmp * t1250 * 1.0E+8;
  t1380 = muDoubleScalarAbs(t1379);
  t1384 = muDoubleScalarAbs(t1383);
  t1389 = muDoubleScalarAbs(t1387);
  t1354 = muDoubleScalarAbs(t1353);
  t1362_tmp = t1361_tmp * t1201;
  t1362 = (-(t1198 * (t1260_tmp - 1.0)) + t1302) +
          t1362_tmp * t1237 * 493.82716049382719;
  t1365 = muDoubleScalarAbs(t1361);
  t1375 = muDoubleScalarAbs(t1372);
  t1363 = muDoubleScalarAbs(t1362);
  d = (t1037 * t1037 + t1044 * t1044) + t926 * t926;
  st.site = &md_emlrtRSI;
  if (d < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &st, &h_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
  }
  d = muDoubleScalarSqrt(d);
  t1378 = 1.0 / d;
  d = (t1380 * t1380 + t1384 * t1384) + t1389 * t1389;
  st.site = &od_emlrtRSI;
  if (d < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &st, &h_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
  }
  d = muDoubleScalarSqrt(d);
  t1396 = 1.0 / d;
  d = (t1358 * t1358 + t1370 * t1370) + t1375 * t1375;
  st.site = &pd_emlrtRSI;
  if (d < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &st, &h_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
  }
  d = muDoubleScalarSqrt(d);
  t1394 = 1.0 / d;
  d = (t1354 * t1354 + t1365 * t1365) + t1363 * t1363;
  st.site = &qd_emlrtRSI;
  if (d < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &st, &h_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
  }
  d = muDoubleScalarSqrt(d);
  t1392 = 1.0 / d;
  D_dn3[0] = 0.0;
  D_dn3[1] = 0.0;
  D_dn3[2] = 0.0;
  D_dn3[3] = 0.0;
  D_dn3[4] = 0.0;
  D_dn3[5] = 0.0;
  D_dn3[6] = 0.0;
  D_dn3[7] = t1312;
  D_dn3[8] = -t1325;
  D_dn3[9] = 0.0;
  D_dn3[10] = 0.0;
  D_dn3[11] = 0.0;
  D_dn3[12] = 0.0;
  D_dn3[13] = 0.0;
  D_dn3[14] = t1312;
  D_dn3[15] = -t1325;
  D_dn3[16] = 0.0;
  D_dn3[17] = 0.0;
  D_dn3[18] = 0.0;
  D_dn3[19] = 0.0;
  D_dn3[20] = 0.0;
  d = t910 * t1005;
  d1 = muDoubleScalarSign(t1328) * t926;
  d2 = t1037 * muDoubleScalarSign(t1022);
  d3 = t910 * t986;
  d4 = t1044 * muDoubleScalarSign(t901);
  d5 = t1242_tmp * t1154;
  d6 = ct[172] * t1005 * t1154;
  D_dn3[21] = t1378 *
              ((d2 *
                    (((-ct[172] * t972 * t1154 * a_tmp + d6 * a_tmp) +
                      t1320_tmp * t980 * t1154) +
                     t1320_tmp * t986 * t1154) *
                    2.0 +
                d4 *
                    (((((t980 * (t1218 - 1.0) + t986 * t1218) - t986 * t1219) +
                       d5 * a_tmp) -
                      t910 * t972 * t1154 * a_tmp) +
                     d * t1154 * a_tmp * 2.0) *
                    2.0) +
               d1 *
                   (((((t1017 + t972 * (t1219 - 1.0)) + t1005 * t1218) -
                      t1005 * t1219) -
                     t910 * t980 * t1154 * a_tmp) -
                    d3 * t1154 * a_tmp * 2.0) *
                   2.0) *
              -0.5;
  t1035 = (-ct[312] + t1066) + ct[207] * a_tmp * 2.0;
  t923 = ct[172] * ct[207];
  t1056 = ct[172] * ct[206];
  t1166_tmp = ct[172] * ct[232];
  t881_tmp = ct[181] * ct[276];
  t907 = ct[207] * t910;
  t1325 = ct[197] * t1155;
  t1312 = ct[172] * t986 * t1155 * a_tmp;
  t1176_tmp = t979 * t1155;
  t1051 = t928 * t1155;
  d3 = d3 * t1155 * a_tmp;
  t1016 = t978 * t1155;
  t979 = t929 * t1155;
  t978 = t1242_tmp * t1155 * a_tmp;
  d = d * t1155 * a_tmp;
  D_dn3[22] =
      t1378 *
      ((d1 *
            (((((((((ct[259] * (t1219 - 1.0) +
                     t986 * (t979 * t1035 - ct[207] * t1154 * a_tmp * 2.0)) -
                    t1166_tmp * t1154 * a_tmp) -
                   t881_tmp * t1154 * a_tmp) +
                  ct[256] * t910 * t1154 * a_tmp) +
                 ct[206] * t1005 * t1154 * a_tmp) +
                t923 * ct[276] * t1154) +
               t907 * t1005 * t1154) -
              t978 * t1035) -
             d * t1035) *
            2.0 +
        d2 *
            (((((((((ct[232] * t1196 +
                     ct[276] * (ct[312] * t1154 + t1325 * t1035)) -
                    ct[172] * ct[259] * t1154 * a_tmp) -
                   ct[181] * t986 * t1154 * a_tmp) -
                  ct[172] * ct[256] * t910 * t1154) +
                 t923 * t986 * t1154) -
                t1056 * t1005 * t1154) +
               ct[181] * t910 * t1005 * t1154) -
              t1312 * t1035) +
             t1176_tmp * t1035) *
            2.0) +
       d4 *
           (((((((((ct[256] * (t1218 - 1.0) +
                    t1005 * (t1066 * t1154 - t1051 * t1035)) -
                   ct[206] * t986 * t1154 * a_tmp) +
                  ct[259] * t910 * t1154 * a_tmp) +
                 t1056 * ct[276] * t1154) -
                t1166_tmp * t910 * t1154) -
               t881_tmp * t910 * t1154) -
              t907 * t986 * t1154) +
             d3 * t1035) -
            t1016 * t1035) *
           -2.0) *
      -0.5;
  t1035 = ct[1] * ct[87];
  t923 = ct[36] * ct[179];
  t1056 = t923 * a_tmp;
  t1166_tmp = (ct[260] - t1064) + t1056 * 2.0;
  t881_tmp = ct[36] * ct[172] * ct[179];
  t907 = t1064_tmp * ct[172];
  t1037 = t1035 * ct[172];
  t901 = t923 * t910;
  t1044 = t1037 * t1154;
  t1017 = t1035 * ct[276];
  D_dn3[23] = t1378 *
              ((d2 *
                    (((((((((ct[276] * (t1325 * t1166_tmp - t1044 * 0.42) -
                             t1035 * t1196 * 0.21) -
                            t1312 * t1166_tmp) +
                           t1176_tmp * t1166_tmp) +
                          t1035 * t986 * t1154 * a_tmp * 0.21) -
                         t881_tmp * t1154 * a_tmp) -
                        t1035 * t910 * t1005 * t1154 * 0.21) +
                       t907 * t910 * t1154) +
                      t881_tmp * t986 * t1154) +
                     t907 * t1005 * t1154) *
                    2.0 +
                d4 *
                    (((((((((t1005 * (t1064 * t1154 + t1051 * t1166_tmp) +
                             t1064_tmp * (t1218 - 1.0)) -
                            d3 * t1166_tmp) +
                           t1016 * t1166_tmp) -
                          t901 * t1154 * a_tmp) -
                         t1064_tmp * t986 * t1154 * a_tmp) -
                        t1037 * t910 * t1154 * 0.21) -
                       t1017 * t910 * t1154 * 0.21) +
                      t907 * ct[276] * t1154) +
                     t901 * t986 * t1154) *
                    2.0) +
               d1 *
                   (((((((((t986 * (t1056 * -2.0 * t1154 + t979 * t1166_tmp) +
                            t923 * (t1219 - 1.0)) -
                           t978 * t1166_tmp) -
                          d * t1166_tmp) +
                         t1044 * a_tmp * 0.21) +
                        t1017 * t1154 * a_tmp * 0.21) -
                       b_t1064_tmp * t1154 * a_tmp) -
                      t1064_tmp * t1005 * t1154 * a_tmp) +
                     t881_tmp * ct[276] * t1154) +
                    t901 * t1005 * t1154) *
                   2.0) /
              2.0;
  t1035 = ct[88] - ct[216];
  t923 = t910 * t1035;
  t1056 = (ct[311] + ct[280] * a_tmp * 2.0) + t923 * 2.0;
  t1166_tmp = ct[280] * t910;
  t881_tmp = ct[172] * ct[180];
  t907 = ct[180] * ct[276];
  t1037 = t881_tmp * t1154;
  t901 = ct[172] * ct[280];
  D_dn3[24] =
      t1378 *
      ((d1 *
            (((((((((ct[280] * (t1219 - 1.0) -
                     t986 * (ct[280] * t1154 * a_tmp * 2.0 - t979 * t1056)) +
                    t1037 * a_tmp) +
                   t907 * t1154 * a_tmp) +
                  t1242_tmp * ct[280] * t1154) +
                 t1166_tmp * t1005 * t1154) +
                t910 * t1154 * a_tmp * t1035) +
               t1005 * t1154 * a_tmp * t1035) -
              t978 * t1056) -
             d * t1056) *
            -2.0 +
        d4 *
            (((((((((-t1005 * (t923 * -2.0 * t1154 + t1051 * t1056) +
                     (t1218 - 1.0) * t1035) +
                    t1166_tmp * t1154 * a_tmp) +
                   t881_tmp * t910 * t1154) +
                  t907 * t910 * t1154) -
                 t1166_tmp * t986 * t1154) -
                t986 * t1154 * a_tmp * t1035) +
               d5 * t1035) +
              d3 * t1056) -
             t1016 * t1056) *
            2.0) +
       d2 *
           (((((((((ct[180] * t1196 - ct[276] * (t1325 * t1056 - t1037 * 2.0)) +
                   t901 * t1154 * a_tmp) -
                  ct[180] * t986 * t1154 * a_tmp) -
                 t901 * t986 * t1154) +
                ct[180] * t910 * t1005 * t1154) +
               t1320_tmp * t1154 * t1035) +
              d6 * t1035) +
             t1312 * t1056) -
            t1176_tmp * t1056) *
           2.0) *
      -0.5;
  D_dn3[25] = 0.0;
  D_dn3[26] = 0.0;
  D_dn3[27] = 0.0;
  d = t1091 * t1095;
  d1 = t1370 * muDoubleScalarSign(t1369);
  d2 = t1358 * muDoubleScalarSign(t1357);
  d3 = muDoubleScalarSign(t1372) * t1375;
  D_dn3[28] =
      t1394 *
      ((d1 *
            (((((t1305 + t1171 * t1270_tmp) + t1176 * t1179 * t1250 * 1.0E+8) -
               t1176 * t1180 * t1250 * 1.0E+8) +
              d * t1177 * t1250 * 2.0) -
             t1166 * ct[58] * ct[59] * t1250 * 1.0E+8) *
            2.0 -
        d3 *
            (((((t1304 + t1166 * (t1268_tmp - 1.0)) +
                t1177 * t1179 * t1250 * 1.0E+8) -
               t1177 * t1180 * t1250 * 1.0E+8) -
              d * t1176 * t1250 * 2.0) -
             ct[58] * t1171 * ct[59] * t1250 * 1.0E+8) *
            2.0) +
       d2 *
           (((ct[29] * t1166 * ct[59] * t1250 * -1.0E+8 +
              t1357_tmp * t1171 * t1250 * 1.0E+8) +
             t1357_tmp * t1176 * t1250 * 1.0E+8) +
            b_t1357_tmp * t1177 * t1250 * 1.0E+8) *
           2.0) *
      -0.5;
  d = ct[28] * ct[29];
  d4 = ct[27] * ct[35];
  d5 = ct[38] * ct[58];
  d6 = ct[37] * ct[59];
  t1035 = t1031 * t1251;
  t923 = c_t1357_tmp * t1251;
  t1056 = d_t1357_tmp * t1251;
  t1166_tmp = t1097 * t1251;
  t881_tmp = b_t1304_tmp * t1251;
  t907 = b_t1369_tmp * t1251;
  t1325 = t1099 * t1251;
  t1312 = t1305_tmp * t1251;
  t1176_tmp = t1372_tmp * t1251;
  D_dn3[29] =
      t1394 *
      ((d1 *
            (((((((((t1104 * t1270_tmp +
                     t1177 * (t1290_tmp * t1250 * 2.0 - t1166_tmp * t1290)) +
                    t1304_tmp * ct[37] * t1250 * 1.0E+8) -
                   d * ct[58] * t1250 * 1.0E+8) -
                  d4 * ct[58] * t1250 * 1.0E+8) -
                 d5 * t1176 * t1250 * 1.0E+8) +
                t1106 * ct[58] * ct[59] * t1250 * 1.0E+8) -
               d6 * t1176 * t1250 * 1.0E+8) -
              t881_tmp * t1290 * 1.0E+8) +
             t907 * t1290 * 1.0E+8) *
            -2.0 +
        d2 *
            (((((((((ct[35] * (t1147 * t1250 + t1035 * t1290) +
                     ct[28] * t1264) -
                    ct[29] * t1104 * ct[58] * t1250 * 1.0E+8) -
                   ct[29] * ct[37] * t1177 * t1250 * 1.0E+8) +
                  ct[29] * ct[38] * t1176 * t1250 * 1.0E+8) -
                 ct[29] * t1106 * ct[59] * t1250 * 1.0E+8) +
                ct[27] * ct[58] * t1177 * t1250 * 1.0E+8) -
               ct[27] * ct[59] * t1176 * t1250 * 1.0E+8) +
              t923 * t1290 * 1.0E+8) -
             t1056 * t1290 * 1.0E+8) *
            2.0) +
       d3 *
           (((((((((t1176 * (t1203 * t1250 - t1325 * t1290) -
                    t1106 * (t1268_tmp - 1.0)) -
                   t1304_tmp * ct[38] * t1250 * 1.0E+8) +
                  d * ct[59] * t1250 * 1.0E+8) +
                 d4 * ct[59] * t1250 * 1.0E+8) -
                d5 * t1177 * t1250 * 1.0E+8) -
               t1104 * ct[58] * ct[59] * t1250 * 1.0E+8) -
              d6 * t1177 * t1250 * 1.0E+8) +
             t1312 * t1290 * 1.0E+8) +
            t1176_tmp * t1290 * 1.0E+8) *
           2.0) *
      -0.5;
  d = ct[29] * ct[304];
  d4 = ct[35] * ct[309];
  d5 = ct[49] * ct[58];
  d6 = ct[50] * ct[59];
  D_dn3[30] =
      t1394 *
      ((d2 *
            (((((((((ct[304] * t1264 +
                     ct[35] * (t1287_tmp * t1250 * 2.0 - t1035 * t1287)) +
                    ct[58] * ct[309] * t1177 * t1250 * 1.0E+8) -
                   ct[59] * ct[309] * t1176 * t1250 * 1.0E+8) +
                  ct[29] * t1121 * ct[58] * t1250 * 1.0E+8) -
                 ct[29] * t1125 * ct[59] * t1250 * 1.0E+8) +
                ct[29] * ct[49] * t1176 * t1250 * 1.0E+8) -
               ct[29] * ct[50] * t1177 * t1250 * 1.0E+8) -
              t923 * t1287 * 1.0E+8) +
             t1056 * t1287 * 1.0E+8) *
            2.0 +
        d1 *
            (((((((((t1121 * t1270_tmp +
                     t1177 * (b_t1287_tmp * t1250 * 2.0 - t1166_tmp * t1287)) +
                    d * ct[58] * t1250 * 1.0E+8) +
                   d4 * ct[58] * t1250 * 1.0E+8) -
                  t1304_tmp * ct[50] * t1250 * 1.0E+8) -
                 t1125 * ct[58] * ct[59] * t1250 * 1.0E+8) +
                d5 * t1176 * t1250 * 1.0E+8) +
               d6 * t1176 * t1250 * 1.0E+8) -
              t881_tmp * t1287 * 1.0E+8) +
             t907 * t1287 * 1.0E+8) *
            2.0) +
       d3 *
           (((((((((-t1176 * (t1211 * t1250 + t1325 * t1287) +
                    t1125 * (t1268_tmp - 1.0)) -
                   d * ct[59] * t1250 * 1.0E+8) -
                  d4 * ct[59] * t1250 * 1.0E+8) +
                 t1304_tmp * ct[49] * t1250 * 1.0E+8) -
                t1121 * ct[58] * ct[59] * t1250 * 1.0E+8) +
               d5 * t1177 * t1250 * 1.0E+8) +
              d6 * t1177 * t1250 * 1.0E+8) +
             t1312 * t1287 * 1.0E+8) +
            t1176_tmp * t1287 * 1.0E+8) *
           -2.0) *
      -0.5;
  d = ct[2] * ct[29];
  d4 = ct[16] * ct[35];
  d5 = ct[43] * ct[58];
  d6 = ct[47] * ct[59];
  t1051 = t1153_tmp - ct[11];
  t1016 = t1357_tmp * t1250;
  t979 = t1369_tmp * t1250;
  D_dn3[31] = t1394 *
              ((d1 *
                    (((((((((t1177 * (t1210 * t1250 - t1166_tmp * t1288) -
                             t1270_tmp * t1051) +
                            d * ct[58] * t1250 * 1.0E+8) +
                           d4 * ct[58] * t1250 * 1.0E+8) -
                          t1304_tmp * ct[47] * t1250 * 1.0E+8) -
                         t1103 * ct[58] * ct[59] * t1250 * 1.0E+8) +
                        d5 * t1176 * t1250 * 1.0E+8) +
                       d6 * t1176 * t1250 * 1.0E+8) -
                      t881_tmp * t1288 * 1.0E+8) +
                     t907 * t1288 * 1.0E+8) *
                    2.0 +
                d3 *
                    (((((((((-t1176 * (t1202 * t1250 + t1325 * t1288) +
                             t1103 * (t1268_tmp - 1.0)) -
                            d * ct[59] * t1250 * 1.0E+8) +
                           t1304_tmp * ct[43] * t1250 * 1.0E+8) -
                          d4 * ct[59] * t1250 * 1.0E+8) +
                         d5 * t1177 * t1250 * 1.0E+8) +
                        d6 * t1177 * t1250 * 1.0E+8) +
                       t979 * t1051 * 1.0E+8) +
                      t1312 * t1288 * 1.0E+8) +
                     t1176_tmp * t1288 * 1.0E+8) *
                    -2.0) +
               d2 *
                   (((((((((ct[2] * t1264 + ct[35] * (t1288_tmp * t1250 * 2.0 -
                                                      t1035 * t1288)) -
                           ct[29] * t1103 * ct[59] * t1250 * 1.0E+8) +
                          ct[29] * ct[43] * t1176 * t1250 * 1.0E+8) +
                         ct[16] * ct[58] * t1177 * t1250 * 1.0E+8) -
                        ct[29] * ct[47] * t1177 * t1250 * 1.0E+8) -
                       ct[16] * ct[59] * t1176 * t1250 * 1.0E+8) -
                      t1016 * t1051 * 1.0E+8) -
                     t923 * t1288 * 1.0E+8) +
                    t1056 * t1288 * 1.0E+8) *
                   2.0) /
              2.0;
  d = ct[253] - ct[9];
  d4 = ct[29] * ct[314];
  d5 = ct[35] * ct[314];
  d6 = ct[39] * ct[58];
  t1051 = ct[40] * ct[59];
  D_dn3[32] =
      t1394 *
      ((d2 *
            (((((((((ct[314] * t1264 +
                     ct[35] * (t1285_tmp * t1250 * 2.0 - t1035 * t1285)) +
                    ct[58] * ct[314] * t1177 * t1250 * 1.0E+8) -
                   ct[59] * ct[314] * t1176 * t1250 * 1.0E+8) +
                  ct[29] * t1108 * ct[59] * t1250 * 1.0E+8) -
                 ct[29] * ct[39] * t1176 * t1250 * 1.0E+8) -
                ct[29] * ct[40] * t1177 * t1250 * 1.0E+8) -
               t1016 * d * 1.0E+8) -
              t923 * t1285 * 1.0E+8) +
             t1056 * t1285 * 1.0E+8) *
            -2.0 +
        d1 *
            (((((((((t1270_tmp * d +
                     t1177 * (t1166_tmp * t1285 +
                              t1091 * t1250 * b_t1285_tmp * 2.0)) -
                    d4 * ct[58] * t1250 * 1.0E+8) -
                   d5 * ct[58] * t1250 * 1.0E+8) +
                  t1304_tmp * ct[40] * t1250 * 1.0E+8) -
                 t1108 * ct[58] * ct[59] * t1250 * 1.0E+8) +
                d6 * t1176 * t1250 * 1.0E+8) -
               t1051 * t1176 * t1250 * 1.0E+8) +
              t881_tmp * t1285 * 1.0E+8) -
             t907 * t1285 * 1.0E+8) *
            2.0) +
       d3 *
           (((((((((t1176 * (t1182 * t1250 - t1325 * t1285) -
                    t1108 * (t1268_tmp - 1.0)) -
                   d4 * ct[59] * t1250 * 1.0E+8) -
                  d5 * ct[59] * t1250 * 1.0E+8) -
                 t1304_tmp * ct[39] * t1250 * 1.0E+8) -
                d6 * t1177 * t1250 * 1.0E+8) +
               t1051 * t1177 * t1250 * 1.0E+8) +
              t979 * d * 1.0E+8) +
             t1312 * t1285 * 1.0E+8) +
            t1176_tmp * t1285 * 1.0E+8) *
           2.0) *
      -0.5;
  D_dn3[33] = 0.0;
  D_dn3[34] = 0.0;
  d = t1161 * t1213;
  d1 = t1380 * muDoubleScalarSign(t1379);
  d2 = t1140 * t1161;
  d3 = t1384 * muDoubleScalarSign(t1383);
  d4 = muDoubleScalarSign(t1387) * t1389;
  d5 = t1161 * t1300;
  d6 = d5 * t1351_tmp;
  t1035 = d * t1300;
  t923 = d2 * t1300;
  D_dn3[35] =
      t1396 *
      ((d1 *
            (((t1161 * t1208 * t1300 * b_a_tmp - t1035 * b_a_tmp) +
              t1161 * t1204 * t1300 * c_a_tmp) +
             d6 * c_a_tmp) *
            -2.0 +
        d3 *
            (((((t1348 + t1351 * 2.0) + t1208 * (t1335 - 1.0)) -
               t1213 * t1335) +
              t1213 * t1337) +
             t1204 * t1300 * b_a_tmp * c_a_tmp) *
            2.0) +
       d4 *
           (((((t1352 * -2.0 + t1204 * (t1337 - 1.0)) - t1335 * t1351_tmp) +
              t1337 * t1351_tmp) +
             t923 * b_a_tmp) +
            t1208 * t1300 * b_a_tmp * c_a_tmp) *
           2.0) *
      -0.5;
  t1056 = ((ct[108] - ct[242]) + ct[255]) + ct[67] * (ct[97] - ct[163]);
  t1166_tmp = t1161 * t1056;
  t881_tmp = (t1278_tmp * -2.0 + t1188 * b_a_tmp * 2.0) + t1166_tmp * 2.0;
  t907 = t1122 * t1161 * t1300;
  t1325 = t1140 * t1300;
  t1312 = t1188 * t1300;
  t1176_tmp = d2 * t1301;
  t1051 = t1176_tmp * t881_tmp;
  t1016 = t1164 * t1301;
  d *= t1301;
  t979 = t1230 * t1301;
  t978 = t1233 * t1301;
  t1037 = -t1161 * t1301 * t1351_tmp;
  t901 = -t1213 * t1301;
  t1044 = -t1301 * t1351_tmp;
  D_dn3[36] =
      t1396 *
      ((d1 *
            ((((((((t1122 * t1324 +
                    t1140 * (t1166_tmp * -2.0 * t1300 + t1016 * t881_tmp)) -
                   t1161 * t1189 * t1213 * t1300) -
                  t1149 * t1161 * t1300 * b_a_tmp) +
                 t1148 * t1161 * t1300 * c_a_tmp) +
                t1161 * t1188 * t1300 * t1351_tmp) +
               t1352_tmp * t1056 * c_a_tmp) +
              b_t1351_tmp * t1056) +
             (t1037 * t881_tmp * b_a_tmp - d * t881_tmp * c_a_tmp)) *
            -2.0 +
        d3 *
            (((((((((t1312 * b_a_tmp * 2.0 - t979 * t881_tmp) * t1351_tmp -
                    t1149 * (t1335 - 1.0)) -
                   d2 * t1188 * t1300) +
                  t907 * b_a_tmp) -
                 t1189 * t1213 * t1300 * b_a_tmp) +
                t1188 * t1213 * t1300 * c_a_tmp) -
               t1325 * b_a_tmp * t1056) +
              t1148 * t1300 * b_a_tmp * c_a_tmp) +
             (t901 * t881_tmp * b_a_tmp * c_a_tmp + t1051 * b_a_tmp)) *
            2.0) +
       d4 *
           ((((((((t1148 * (t1337 - 1.0) -
                   t1213 * (t1278_tmp * 2.0 * t1300 + t978 * t881_tmp)) +
                  d2 * t1189 * t1300) +
                 t907 * c_a_tmp) -
                t1149 * t1300 * b_a_tmp * c_a_tmp) -
               t1325 * t1056 * c_a_tmp) -
              t1189 * t1300 * t1351_tmp * b_a_tmp) +
             t1312 * t1351_tmp * c_a_tmp) +
            (t1044 * t881_tmp * b_a_tmp * c_a_tmp + t1051 * c_a_tmp)) *
           2.0) /
      2.0;
  t1056 = ((ct[106] - ct[109]) + ct[124]) + ct[267];
  t1166_tmp = t1161 * t1056;
  t881_tmp = t1195 * b_a_tmp;
  t907 = t1187 * c_a_tmp;
  t1312 = (t1166_tmp * 2.0 + t881_tmp * 2.0) + t907 * 2.0;
  t1051 = t1053 * t1161 * t1300;
  t1017 = t1325 * t1056;
  t1383 = t1161 * t1301;
  t1328 = t1213 * t1301;
  t926 = t1301 * t1351_tmp;
  t1022 = t1383 * t1351_tmp;
  D_dn3[37] =
      t1396 *
      ((d1 *
            ((((((((t1053 * t1324 -
                    t1140 * (t1166_tmp * -2.0 * t1300 + t1016 * t1312)) -
                   t1161 * t1187 * t1213 * t1300) +
                  t1134 * t1161 * t1300 * b_a_tmp) +
                 t1129 * t1161 * t1300 * c_a_tmp) -
                t1161 * t1195 * t1300 * t1351_tmp) -
               t1352_tmp * t1056 * c_a_tmp) -
              t1351_tmp_tmp * t1056 * b_a_tmp) +
             (t1022 * t1312 * b_a_tmp + d * t1312 * c_a_tmp)) *
            -2.0 +
        d3 *
            ((((((((t1134 * (t1335 - 1.0) +
                    (t881_tmp * -2.0 * t1300 + t979 * t1312) * t1351_tmp) +
                   d2 * t1195 * t1300) +
                  t1051 * b_a_tmp) -
                 t1187 * t1213 * t1300 * b_a_tmp) -
                t1195 * t1213 * t1300 * c_a_tmp) +
               t1017 * b_a_tmp) +
              t1129 * t1300 * b_a_tmp * c_a_tmp) +
             (t1348_tmp_tmp * t1301 * t1312 * b_a_tmp +
              t1328 * t1312 * b_a_tmp * c_a_tmp)) *
            2.0) +
       d4 *
           ((((((((t1129 * (t1337 - 1.0) +
                   t1213 * (t907 * -2.0 * t1300 + t978 * t1312)) +
                  d2 * t1187 * t1300) +
                 t1051 * c_a_tmp) +
                t1017 * c_a_tmp) +
               t1134 * t1300 * b_a_tmp * c_a_tmp) -
              t1187 * t1300 * t1351_tmp * b_a_tmp) -
             t1195 * t1300 * t1351_tmp * c_a_tmp) +
            (t926 * t1312 * b_a_tmp * c_a_tmp - t1176_tmp * t1312 * c_a_tmp)) *
           2.0) /
      2.0;
  t1056 = ((ct[71] + ct[231]) + ct[183]) - ct[241];
  t1166_tmp = (t1153_tmp - t881) + t1273_tmp;
  t881_tmp = t1112 * t1140 * t1300;
  t907 = d5 * t1056;
  t1312 = t1176_tmp * t1342;
  D_dn3[38] =
      t1396 *
      ((d1 *
            (((((((((-t1140 * (t1245 * t1300 + t1016 * t1342) + t1324 * t1056) -
                    t1145 * t1161 * t1300 * b_a_tmp) +
                   t1112 * t1213 * t1300 * c_a_tmp) +
                  d5 * t1166_tmp * c_a_tmp) +
                 t1112 * t1300 * t1351_tmp * b_a_tmp) -
                d6 * t1342_tmp) -
               t1035 * b_t1273_tmp) +
              d * t1342 * c_a_tmp) +
             t1383 * t1342 * t1351_tmp * b_a_tmp) *
            2.0 -
        d3 *
            ((((((((t979 * t1342 - t1300 * t1342_tmp * b_a_tmp * 2.0) *
                       t1351_tmp -
                   t1145 * (t1335 - 1.0)) -
                  t881_tmp * b_a_tmp) +
                 t923 * t1342_tmp) +
                t907 * b_a_tmp) -
               t1352_tmp * t1342_tmp * c_a_tmp) +
              t1300 * b_a_tmp * t1166_tmp * c_a_tmp) +
             ((-t1213 * t1300 * b_t1273_tmp * b_a_tmp - t1312 * b_a_tmp) +
              t1328 * t1342 * b_a_tmp * c_a_tmp)) *
            2.0) +
       d4 *
           ((((((((t1213 * (t1273 * t1300 - t978 * t1342) -
                   (t1337 - 1.0) * t1166_tmp) +
                  t1351_tmp_tmp * b_t1273_tmp * b_a_tmp) +
                 t881_tmp * c_a_tmp) -
                t907 * c_a_tmp) +
               t1145 * t1300 * b_a_tmp * c_a_tmp) +
              t1351_tmp_tmp * t1342_tmp * c_a_tmp) -
             t923 * b_t1273_tmp) +
            (-t1301 * t1342 * t1351_tmp * b_a_tmp * c_a_tmp +
             t1312 * c_a_tmp)) *
           2.0) /
      2.0;
  d6 = (t1247_tmp * 2.0 - t1280) + t1192 * b_a_tmp * 2.0;
  t1056 = t1118 * t1140 * t1300;
  t1166_tmp = t1192 * t1300;
  t881_tmp = t1176_tmp * d6;
  t907 = ct[98] - ct[190];
  D_dn3[39] =
      t1396 *
      ((d1 *
            (((((((((t1140 * (t1016 * d6 - t1247_tmp * t1300 * 2.0) +
                     ct[67] * t1324 * t907) -
                    t1035 * t1280_tmp) -
                   t1052 * t1161 * t1300 * b_a_tmp) -
                  ct[21] * t1161 * t1300 * c_a_tmp) +
                 t1118 * t1213 * t1300 * c_a_tmp) +
                t1161 * t1192 * t1300 * t1351_tmp) +
               t1118 * t1300 * t1351_tmp * b_a_tmp) -
              d * d6 * c_a_tmp) +
             t1037 * d6 * b_a_tmp) *
            2.0 +
        d3 *
            (((((((((t1052 * (t1335 - 1.0) +
                     (t979 * d6 - t1166_tmp * b_a_tmp * 2.0) * t1351_tmp) +
                    d2 * t1192 * t1300) +
                   t1352_tmp * t1280_tmp * b_a_tmp) +
                  ct[295] * t1161 * t1300 * b_a_tmp) +
                 t1056 * b_a_tmp) -
                t1192 * t1213 * t1300 * c_a_tmp) +
               ct[21] * t1300 * b_a_tmp * c_a_tmp) -
              t881_tmp * b_a_tmp) +
             t1328 * d6 * b_a_tmp * c_a_tmp) *
            2.0) -
       d4 *
           ((((((-t1213 * (t1280 * t1300 + t978 * d6) +
                 ct[67] * (t1337 - 1.0) * t1280_tmp_tmp) +
                t923 * t1280_tmp) -
               t1056 * c_a_tmp) -
              t1351_tmp_tmp * t1280_tmp * b_a_tmp) +
             t1166_tmp * t1351_tmp * c_a_tmp) +
            (((ct[67] * t1300 * t1052_tmp * b_a_tmp * c_a_tmp +
               t881_tmp * c_a_tmp) -
              t926 * d6 * b_a_tmp * c_a_tmp) +
             ct[67] * t1161 * t1300 * t907 * c_a_tmp)) *
           2.0) /
      2.0;
  d2 = t1142 * t1161;
  d6 = t1135 * t1161 * t1300;
  t1035 = (t1235_tmp * -2.0 + t1135 * b_a_tmp * 2.0) + t1142 * c_a_tmp * 2.0;
  t923 = t1142 * t1300;
  t1056 = t1135 * t1300;
  t1166_tmp = t1056 * b_a_tmp;
  t881_tmp = t1325 * t1235_tmp_tmp;
  d5 *= t1235_tmp_tmp;
  t907 = t1176_tmp * t1035;
  D_dn3[40] =
      t1396 *
      ((d1 *
            (((((((((-t1140 * (t1235_tmp * 2.0 * t1300 + t1016 * t1035) -
                     t1324 * t1235_tmp_tmp) -
                    d2 * t1213 * t1300) +
                   d6 * b_a_tmp) +
                  d2 * t1300 * c_a_tmp) -
                 d6 * t1351_tmp) +
                t1352_tmp * t1235_tmp_tmp * c_a_tmp) +
               t1351_tmp_tmp * t1235_tmp_tmp * b_a_tmp) +
              t1022 * t1035 * b_a_tmp) +
             d * t1035 * c_a_tmp) *
            2.0 +
        d3 *
            (((((((((t1166_tmp * 2.0 - t979 * t1035) * t1351_tmp -
                    t1135 * (t1335 - 1.0)) -
                   t1135 * t1140 * t1161 * t1300) +
                  t1142 * t1213 * t1300 * b_a_tmp) +
                 t1135 * t1213 * t1300 * c_a_tmp) -
                t923 * b_a_tmp * c_a_tmp) +
               t881_tmp * b_a_tmp) +
              d5 * b_a_tmp) +
             (t901 * t1035 * b_a_tmp * c_a_tmp + t907 * b_a_tmp)) *
            2.0) +
       d4 *
           ((((((((-t1142 * (t1337 - 1.0) +
                   t1213 * (t923 * c_a_tmp * 2.0 - t978 * t1035)) -
                  t1140 * t1142 * t1161 * t1300) -
                 t1166_tmp * c_a_tmp) +
                t923 * t1351_tmp * b_a_tmp) +
               t1056 * t1351_tmp * c_a_tmp) +
              t881_tmp * c_a_tmp) +
             d5 * c_a_tmp) +
            (t1044 * t1035 * b_a_tmp * c_a_tmp + t907 * c_a_tmp)) *
           2.0) /
      2.0;
  D_dn3[41] = 0.0;
  d = t1083 * t1084;
  d1 = t1365 * muDoubleScalarSign(t1361);
  d2 = t1354 * muDoubleScalarSign(t1353);
  d3 = t1363 * muDoubleScalarSign(t1362);
  D_dn3[42] = t1392 *
              ((d1 *
                    (((((t1302 + t1194 * t1262_tmp) -
                        ct[57] * t1198 * t1237 * 493.82716049382719) +
                       t1172 * t1198 * t1237 * 493.82716049382719) -
                      d * t1201 * t1237 * 2.0) -
                     t1361_tmp * t1191 * t1237 * 493.82716049382719) *
                    2.0 -
                d3 *
                    (((((t1303 + t1191 * (t1260_tmp - 1.0)) -
                        ct[57] * t1201 * t1237 * 493.82716049382719) +
                       t1172 * t1201 * t1237 * 493.82716049382719) +
                      d * t1198 * t1237 * 2.0) -
                     t1361_tmp * t1194 * t1237 * 493.82716049382719) *
                    2.0) +
               d2 *
                   (((t1353_tmp * t1191 * t1237 * -493.82716049382719 +
                      b_t1353_tmp * t1194 * t1237 * 493.82716049382719) +
                     t1353_tmp * t1201 * t1237 * 493.82716049382719) +
                    b_t1353_tmp * t1198 * t1237 * 493.82716049382719) *
                   2.0) *
              -0.5;
  d = ct[23] * ct[31];
  d4 = ct[23] * ct[33];
  d5 = ct[23] * ct[45];
  d6 = ct[25] * ct[51];
  t1035 = ct[31] * ct[54];
  t923 = ct[33] * ct[56];
  t1056 = t1088 * t1238;
  t1166_tmp = b_t1302_tmp * t1238;
  t881_tmp = t1362_tmp * t1238;
  t907 = t1087 * t1238;
  t1325 = t1303_tmp * t1238;
  t1312 = b_t1361_tmp * t1238;
  t1176_tmp = t962 * t1238;
  t1051 = c_t1353_tmp * t1238;
  t1016 = d_t1353_tmp * t1238;
  D_dn3[43] =
      t1392 *
      ((d1 *
            (((((((((t1201 * (t1174 * t1237 + t907 * t1281) +
                     t1136 * t1262_tmp) +
                    d * ct[51] * t1237 * 493.82716049382719) -
                   d5 * ct[56] * t1237 * 493.82716049382719) -
                  d6 * ct[56] * t1237 * 493.82716049382719) -
                 t1035 * t1198 * t1237 * 493.82716049382719) -
                t923 * t1198 * t1237 * 493.82716049382719) +
               t1138 * ct[54] * ct[56] * t1237 * 493.82716049382719) +
              t1325 * t1281 * 493.82716049382719) -
             t1312 * t1281 * 493.82716049382719) *
            2.0 +
        d2 *
            (((((((((ct[45] * t1258 +
                     ct[51] * (t1281_tmp * t1237 * 2.0 - t1176_tmp * t1281)) -
                    d * t1201 * t1237 * 493.82716049382719) +
                   d4 * t1198 * t1237 * 493.82716049382719) -
                  ct[23] * t1138 * ct[54] * t1237 * 493.82716049382719) -
                 ct[23] * t1136 * ct[56] * t1237 * 493.82716049382719) -
                ct[25] * ct[54] * t1198 * t1237 * 493.82716049382719) +
               ct[25] * ct[56] * t1201 * t1237 * 493.82716049382719) +
              t1051 * t1281 * 493.82716049382719) -
             t1016 * t1281 * 493.82716049382719) *
            -2.0) +
       d3 *
           (((((((((t1138 * (t1260_tmp - 1.0) +
                    t1198 * (b_t1281_tmp * t1237 * 2.0 - t1056 * t1281)) +
                   d4 * ct[51] * t1237 * 493.82716049382719) -
                  d5 * ct[54] * t1237 * 493.82716049382719) -
                 d6 * ct[54] * t1237 * 493.82716049382719) +
                t1035 * t1201 * t1237 * 493.82716049382719) +
               t923 * t1201 * t1237 * 493.82716049382719) +
              t1136 * ct[54] * ct[56] * t1237 * 493.82716049382719) +
             t1166_tmp * t1281 * 493.82716049382719) +
            t881_tmp * t1281 * 493.82716049382719) *
           2.0) /
      2.0;
  d = ct[51] * ct[306];
  d4 = ct[15] * ct[23];
  d5 = ct[44] * ct[56];
  d6 = ct[48] * ct[54];
  t1035 = (ct[175] - ct[240]) + ct[67] * (ct[157] + ct[199]);
  t923 = ct[23] * ct[44];
  t979 = ct[23] * ct[48];
  D_dn3[44] =
      t1392 *
      ((d3 *
            (((((((((t1198 * (t1193 * t1237 - t1056 * t1284) -
                     (t1260_tmp - 1.0) * t1035) -
                    d * ct[54] * t1237 * 493.82716049382719) +
                   d4 * ct[54] * t1237 * 493.82716049382719) +
                  t923 * ct[51] * t1237 * 493.82716049382719) -
                 t1141 * ct[54] * ct[56] * t1237 * 493.82716049382719) +
                d5 * t1201 * t1237 * 493.82716049382719) -
               d6 * t1201 * t1237 * 493.82716049382719) +
              t1166_tmp * t1284 * 493.82716049382719) +
             t881_tmp * t1284 * 493.82716049382719) *
            -2.0 +
        d1 *
            (((((((((t1141 * t1262_tmp +
                     t1201 * (t1284_tmp * t1237 * 2.0 - t907 * t1284)) +
                    d * ct[56] * t1237 * 493.82716049382719) -
                   d4 * ct[56] * t1237 * 493.82716049382719) +
                  t979 * ct[51] * t1237 * 493.82716049382719) +
                 d5 * t1198 * t1237 * 493.82716049382719) -
                d6 * t1198 * t1237 * 493.82716049382719) +
               t1361_tmp * t1237 * t1035 * 493.82716049382719) -
              t1325 * t1284 * 493.82716049382719) +
             t1312 * t1284 * 493.82716049382719) *
            2.0) +
       d2 *
           (((((((((-ct[15] * t1258 -
                    ct[51] * (t1176_tmp * t1284 +
                              t940 * t1237 * b_t1284_tmp * 2.0)) -
                   ct[54] * ct[306] * t1198 * t1237 * 493.82716049382719) +
                  ct[56] * ct[306] * t1201 * t1237 * 493.82716049382719) +
                 ct[23] * t1141 * ct[56] * t1237 * 493.82716049382719) +
                t923 * t1198 * t1237 * 493.82716049382719) +
               t979 * t1201 * t1237 * 493.82716049382719) +
              t1353_tmp * t1237 * t1035 * 493.82716049382719) +
             t1051 * t1284 * 493.82716049382719) -
            t1016 * t1284 * 493.82716049382719) *
           2.0) /
      2.0;
  d = t940 * (ct[116] - ct[127]);
  d4 = (-t1181 + t1184) + d * 2.0;
  d5 = ct[23] * ct[41];
  d6 = ct[23] * ct[30];
  t1035 = ct[23] * ct[46];
  t923 = ct[17] * ct[51];
  t979 = ct[41] * ct[56];
  t978 = ct[46] * ct[54];
  D_dn3[45] =
      t1392 *
      ((d2 *
            (((((((((ct[30] * t1258 +
                     ct[51] * (d * -2.0 * t1237 + t1176_tmp * d4)) -
                    ct[23] * t1146 * ct[54] * t1237 * 493.82716049382719) +
                   d5 * t1198 * t1237 * 493.82716049382719) -
                  ct[23] * t1153 * ct[56] * t1237 * 493.82716049382719) -
                 t1035 * t1201 * t1237 * 493.82716049382719) -
                ct[17] * ct[54] * t1198 * t1237 * 493.82716049382719) +
               ct[17] * ct[56] * t1201 * t1237 * 493.82716049382719) -
              t1051 * d4 * 493.82716049382719) +
             t1016 * d4 * 493.82716049382719) *
            -2.0 +
        d3 *
            (((((((((t1146 * (t1260_tmp - 1.0) +
                     t1198 * (t1181 * t1237 + t1056 * d4)) -
                    d6 * ct[54] * t1237 * 493.82716049382719) +
                   d5 * ct[51] * t1237 * 493.82716049382719) -
                  t923 * ct[54] * t1237 * 493.82716049382719) +
                 t1153 * ct[54] * ct[56] * t1237 * 493.82716049382719) +
                t979 * t1201 * t1237 * 493.82716049382719) +
               t978 * t1201 * t1237 * 493.82716049382719) -
              t1166_tmp * d4 * 493.82716049382719) -
             t881_tmp * d4 * 493.82716049382719) *
            2.0) +
       d1 *
           (((((((((t1153 * t1262_tmp + t1201 * (t1184 * t1237 - t907 * d4)) -
                   d6 * ct[56] * t1237 * 493.82716049382719) +
                  t1035 * ct[51] * t1237 * 493.82716049382719) -
                 t923 * ct[56] * t1237 * 493.82716049382719) +
                t1146 * ct[54] * ct[56] * t1237 * 493.82716049382719) -
               t979 * t1198 * t1237 * 493.82716049382719) -
              t978 * t1198 * t1237 * 493.82716049382719) -
             t1325 * d4 * 493.82716049382719) +
            t1312 * d4 * 493.82716049382719) *
           2.0) *
      -0.5;
  d = ct[67] * t1084 * (ct[115] - ct[193]);
  d4 = (-t1069 + t1156_tmp * 2.0) + d * 2.0;
  d5 = ct[161] - ct[238];
  d6 = ct[23] * ct[34];
  t1035 = ct[67] * (ct[144] + ct[247]);
  t923 = ct[23] * ct[308];
  t979 = ct[51] * ct[307];
  t978 = ct[23] * ct[32];
  t1037 = ct[32] * ct[54];
  t901 = ct[34] * ct[56];
  D_dn3[46] =
      t1392 *
      ((d2 *
            (((((((((ct[308] * t1258 +
                     ct[51] * (t1069 * t1237 + t1176_tmp * d4)) -
                    ct[54] * ct[307] * t1198 * t1237 * 493.82716049382719) +
                   ct[56] * ct[307] * t1201 * t1237 * 493.82716049382719) +
                  d6 * t1198 * t1237 * 493.82716049382719) -
                 t978 * t1201 * t1237 * 493.82716049382719) -
                t1035 * ct[23] * ct[56] * t1237 * 493.82716049382719) -
               t1051 * d4 * 493.82716049382719) +
              t1016 * d4 * 493.82716049382719) +
             ct[23] * ct[67] * ct[54] * t1237 * d5 * 493.82716049382719) *
            2.0 +
        d3 *
            (((((((((t1198 * (d * -2.0 * t1237 + t1056 * d4) -
                     ct[67] * (t1260_tmp - 1.0) * d5) -
                    t923 * ct[54] * t1237 * 493.82716049382719) -
                   t979 * ct[54] * t1237 * 493.82716049382719) +
                  d6 * ct[51] * t1237 * 493.82716049382719) +
                 t1037 * t1201 * t1237 * 493.82716049382719) +
                t901 * t1201 * t1237 * 493.82716049382719) +
               t1035 * ct[54] * ct[56] * t1237 * 493.82716049382719) -
              t1166_tmp * d4 * 493.82716049382719) -
             t881_tmp * d4 * 493.82716049382719) *
            -2.0) +
       d1 *
           (((((((((t1201 * (t907 * d4 - t1156_tmp * t1237 * 2.0) -
                    t1035 * t1262_tmp) +
                   t923 * ct[56] * t1237 * 493.82716049382719) +
                  t979 * ct[56] * t1237 * 493.82716049382719) -
                 t978 * ct[51] * t1237 * 493.82716049382719) +
                t1037 * t1198 * t1237 * 493.82716049382719) +
               t901 * t1198 * t1237 * 493.82716049382719) +
              t1325 * d4 * 493.82716049382719) -
             t1312 * d4 * 493.82716049382719) +
            ct[54] * ct[67] * ct[56] * t1237 * d5 * 493.82716049382719) *
           2.0) /
      2.0;
  d = ct[23] * ct[26];
  d4 = ct[24] * ct[51];
  d5 = ct[54] * ct[55];
  d6 = ct[53] * ct[56];
  D_dn3[47] =
      t1392 *
      ((d2 *
            (((((((((-ct[51] * (t1289_tmp * -2.0 * t1237 + t1176_tmp * t1289) +
                     ct[26] * t1258) -
                    ct[23] * t1133 * ct[54] * t1237 * 493.82716049382719) +
                   ct[23] * t1139 * ct[56] * t1237 * 493.82716049382719) +
                  ct[23] * ct[53] * t1198 * t1237 * 493.82716049382719) -
                 ct[24] * ct[54] * t1198 * t1237 * 493.82716049382719) +
                ct[23] * ct[55] * t1201 * t1237 * 493.82716049382719) +
               ct[24] * ct[56] * t1201 * t1237 * 493.82716049382719) +
              t1051 * t1289 * 493.82716049382719) -
             t1016 * t1289 * 493.82716049382719) *
            2.0 +
        d1 *
            (((((((((t1201 * (t1216 * t1237 - t907 * t1289) +
                     t1139 * t1262_tmp) +
                    d * ct[56] * t1237 * 493.82716049382719) +
                   t1302_tmp * ct[55] * t1237 * 493.82716049382719) +
                  d4 * ct[56] * t1237 * 493.82716049382719) -
                 t1133 * ct[54] * ct[56] * t1237 * 493.82716049382719) -
                d5 * t1198 * t1237 * 493.82716049382719) +
               d6 * t1198 * t1237 * 493.82716049382719) -
              t1325 * t1289 * 493.82716049382719) +
             t1312 * t1289 * 493.82716049382719) *
            2.0) +
       d3 *
           (((((((((t1133 * (t1260_tmp - 1.0) +
                    t1198 * (b_t1289_tmp * t1237 * 2.0 - t1056 * t1289)) -
                   d * ct[54] * t1237 * 493.82716049382719) +
                  t1302_tmp * ct[53] * t1237 * 493.82716049382719) -
                 d4 * ct[54] * t1237 * 493.82716049382719) -
                t1139 * ct[54] * ct[56] * t1237 * 493.82716049382719) -
               d5 * t1201 * t1237 * 493.82716049382719) +
              d6 * t1201 * t1237 * 493.82716049382719) +
             t1166_tmp * t1289 * 493.82716049382719) +
            t881_tmp * t1289 * 493.82716049382719) *
           -2.0) *
      -0.5;
  D_dn3[48] = 0.0;
}

void get_ddndq_3(const emlrtStack *sp, const real_T q[7], const real_T o[21],
                 real_T D_dn3[49])
{
  emlrtStack st;
  real_T b_o[439];
  real_T b_o_tmp;
  real_T b_t89_tmp;
  real_T b_t90_tmp;
  real_T c_o_tmp;
  real_T d_o_tmp;
  real_T o_tmp;
  real_T t10;
  real_T t100;
  real_T t101;
  real_T t102;
  real_T t103;
  real_T t104;
  real_T t107;
  real_T t109;
  real_T t11;
  real_T t110;
  real_T t111;
  real_T t112;
  real_T t115;
  real_T t115_tmp;
  real_T t116;
  real_T t12;
  real_T t123;
  real_T t123_tmp;
  real_T t13;
  real_T t14;
  real_T t15;
  real_T t16;
  real_T t17;
  real_T t2;
  real_T t20;
  real_T t208;
  real_T t209;
  real_T t21;
  real_T t22;
  real_T t23;
  real_T t24;
  real_T t25;
  real_T t26;
  real_T t27;
  real_T t28;
  real_T t3;
  real_T t30;
  real_T t31;
  real_T t314;
  real_T t315;
  real_T t318;
  real_T t319;
  real_T t32;
  real_T t321;
  real_T t322;
  real_T t33;
  real_T t34;
  real_T t37;
  real_T t38;
  real_T t39;
  real_T t4;
  real_T t40;
  real_T t409;
  real_T t41;
  real_T t411;
  real_T t42;
  real_T t43;
  real_T t44;
  real_T t45;
  real_T t46;
  real_T t47;
  real_T t48;
  real_T t49;
  real_T t5;
  real_T t50;
  real_T t51;
  real_T t52;
  real_T t527;
  real_T t527_tmp;
  real_T t53;
  real_T t54;
  real_T t541;
  real_T t55;
  real_T t56;
  real_T t57;
  real_T t58;
  real_T t59;
  real_T t6;
  real_T t60;
  real_T t61;
  real_T t62;
  real_T t63;
  real_T t63_tmp;
  real_T t64;
  real_T t65;
  real_T t66;
  real_T t67;
  real_T t68;
  real_T t7;
  real_T t70;
  real_T t75;
  real_T t76;
  real_T t8;
  real_T t89;
  real_T t89_tmp;
  real_T t9;
  real_T t90;
  real_T t90_tmp;
  real_T t93;
  real_T t95;
  real_T t96;
  real_T t97;
  real_T t98;
  st.prev = sp;
  st.tls = sp->tls;
  /* GET_DDDQ_3 */
  /*     D_DN3 = GET_DDDQ_3(O1,O2,O3,Q1,Q2,Q3,Q4,Q5,Q6) */
  /*     This function was generated by the Symbolic Math Toolbox version 8.7.
   */
  /*     08-Dec-2021 21:31:24 */
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
  t20 = o[2] * 50.0;
  t14 = t2 * t2;
  t15 = t3 * t3;
  t16 = t8 * t8;
  t17 = t9 * t9;
  t21 = t2 * t4;
  t22 = t3 * t5;
  t23 = t2 * t10;
  t24 = t4 * t8;
  t25 = t3 * t11;
  t26 = t5 * t9;
  t27 = t8 * t10;
  t28 = t9 * t11;
  t30 = t3 * 0.4;
  t31 = t9 * 0.4;
  t34 = t9 * t10 * t12;
  t61 = t3 * t10 * t12;
  t62 = t4 * t9 * t12;
  t63_tmp = t6 * t9;
  t63 = t63_tmp * t10;
  t89_tmp = o[0] * t3;
  b_t89_tmp = t89_tmp * t9;
  t89 = b_t89_tmp * 100.0;
  t90_tmp = o[1] * t3;
  b_t90_tmp = t90_tmp * t9;
  t90 = b_t90_tmp * 100.0;
  t115_tmp = t2 * t3 * t9;
  t115 = t115_tmp * 17.0;
  t123_tmp = t3 * t8 * t9;
  t123 = t123_tmp * 17.0;
  t527_tmp = t3 * t6 * t10;
  t527 = t527_tmp * 0.0607;
  t32 = t8 * t28;
  t33 = t11 * t27;
  t37 = t3 * t21;
  t38 = t2 * t22;
  t39 = t5 * t21;
  t40 = t4 * t22;
  t41 = t6 * t22;
  t42 = t3 * t23;
  t43 = t3 * t24;
  t44 = t2 * t25;
  t45 = t2 * t26;
  t46 = t8 * t22;
  t47 = t11 * t21;
  t48 = t5 * t23;
  t49 = t5 * t24;
  t50 = t4 * t25;
  t51 = t4 * t26;
  t52 = t6 * t25;
  t53 = t3 * t27;
  t54 = t2 * t28;
  t55 = t8 * t25;
  t56 = t8 * t26;
  t57 = t11 * t23;
  t58 = t11 * t24;
  t59 = t5 * t27;
  t60 = t4 * t28;
  t64 = t12 * t25;
  t65 = t6 * t28;
  t66 = o[0] * t15 * 50.0;
  t67 = o[1] * t15 * 50.0;
  t68 = t15 * t20;
  t70 = t21 * 0.4;
  t75 = t24 * 0.4;
  t76 = t25 * 0.4;
  t93 = t21 * t22;
  t95 = t21 * t25;
  t96 = t22 * t23;
  t97 = t21 * t26;
  t98 = t22 * t24;
  t100 = t23 * t25;
  t101 = t21 * t28;
  t102 = t24 * t25;
  t103 = t22 * t27;
  t104 = t24 * t26;
  t107 = t6 * t10 * t26;
  t109 = t25 * t27;
  t110 = t24 * t28;
  t208 = t9 * t12;
  t111 = t208 * t23;
  t112 = t7 * t10 * t28;
  t116 = t208 * t27;
  t208 = o[1] * t14 * t17 * 50.0;
  t209 = o[0] * t16 * t17 * 50.0;
  t314 = t21 * 0.0607;
  t315 = t22 * 0.0607;
  t318 = t23 * 0.0607;
  t319 = t24 * 0.0607;
  t321 = t27 * 0.0607;
  t322 = t28 * 0.0607;
  t409 = t61 * 0.081;
  t411 = t63 * 0.081;
  t541 = t63 * 0.0607;
  b_o[0] = o[1];
  b_o[1] = t10;
  b_o[2] = t102;
  b_o[3] = t103;
  b_o[4] = t104;
  b_o[5] = t12 * t51;
  b_o[6] = t6 * t60;
  b_o[7] = -t37;
  b_o[8] = t11;
  b_o[9] = t111;
  b_o[10] = t112;
  b_o[11] = -t50;
  b_o[12] = -t51;
  b_o[13] = t115;
  b_o[14] = t116;
  b_o[15] = -t53;
  b_o[16] = -t54;
  b_o[17] = -t56;
  b_o[18] = t12;
  b_o[19] = -t59;
  b_o[20] = -t61;
  b_o[21] = -t64;
  b_o[22] = t123;
  b_o[23] = t32 * 0.4;
  o_tmp = t10 * t28;
  b_o[24] = o_tmp * 0.4;
  b_o[25] = -(t25 * 1900.0);
  b_o_tmp = o[0] * t2;
  b_o[26] = b_o_tmp * t3 * t9 * 50.0;
  c_o_tmp = o[1] * t2;
  b_o[27] = c_o_tmp * t3 * t9 * 50.0;
  b_o[28] = t115_tmp * t20;
  b_o[29] = t13;
  b_o[30] = t2 * t15 * -17.0;
  b_o[31] = t90_tmp * t8 * t9 * 50.0;
  b_o[32] = t123_tmp * t20;
  b_o[33] = t8 * t15 * -17.0;
  d_o_tmp = o[2] * t3;
  b_o[34] = -(d_o_tmp * t9 * 100.0);
  b_o[35] = t2 * t66;
  b_o[36] = t2 * t68;
  b_o[37] = t38 * 0.4;
  b_o[38] = t8 * t67;
  b_o[39] = t8 * t68;
  b_o[40] = t45 * 0.4;
  b_o[41] = t46 * 0.4;
  b_o[42] = t50 * 0.4;
  b_o[43] = t51 * 0.4;
  b_o[44] = -t115;
  b_o[45] = t54 * 0.4;
  b_o[46] = t56 * 0.4;
  b_o[47] = t60 * 0.4;
  b_o[48] = -t123;
  b_o[49] = t21 * 0.21;
  b_o[50] = t22 * 0.19;
  b_o[51] = t22 * 0.21;
  b_o[52] = t34 * 607.0;
  b_o[53] = t12 * t28 * 607.0;
  b_o[54] = t32 * 1900.0;
  b_o[55] = o_tmp * 1900.0;
  b_o[56] = t23 * 0.21;
  b_o[57] = t24 * 0.21;
  b_o[58] = t26 * 0.19;
  b_o[59] = t25 * 0.21;
  b_o[60] = t26 * 0.21;
  b_o[61] = t27 * 0.21;
  t123 = o[2] * t2;
  b_o[62] = t123 * t3 * t9 * -50.0;
  b_o[63] = -(t89_tmp * t8 * t9 * 50.0);
  b_o[64] = d_o_tmp * t8 * t9 * -50.0;
  b_o[65] = -t95;
  b_o[66] = -t97;
  b_o[67] = -t103;
  b_o[68] = -t104;
  b_o[69] = -o[0];
  b_o[70] = t37 * -0.4;
  b_o[71] = t38 * 1900.0;
  b_o[72] = -t109;
  b_o[73] = -t112;
  b_o[74] = -(b_o_tmp * t17 * 50.0);
  b_o[75] = t123 * t17 * -50.0;
  b_o[76] = t527_tmp * 607.0;
  d_o_tmp = t4 * t6 * t9;
  b_o[77] = d_o_tmp * 607.0;
  b_o[78] = -o[2];
  b_o[79] = t12 * t22 * 607.0;
  b_o[80] = t52 * 607.0;
  b_o[81] = t45 * 1900.0;
  b_o[82] = t46 * 1900.0;
  b_o[83] = t50 * 1900.0;
  b_o[84] = t51 * 1900.0;
  b_o[85] = -(o[1] * t8 * t17 * 50.0);
  b_o[86] = o[2] * t8 * t17 * -50.0;
  b_o[87] = t53 * -0.4;
  b_o[88] = t64 * 607.0;
  b_o[89] = t54 * 1900.0;
  b_o[90] = t56 * 1900.0;
  b_o[91] = t60 * 1900.0;
  b_o[92] = -(t25 * 0.19);
  b_o[93] = o[0] * t14 * t17 * 50.0;
  b_o[94] = t208;
  b_o[95] = t209;
  b_o[96] = t21;
  b_o[97] = o[1] * t16 * t17 * 50.0;
  b_o[98] = t28 * t70;
  b_o[99] = t28 * t75;
  b_o[100] = -(t63 * 607.0);
  b_o[101] = t22 * 0.081;
  b_o[102] = t32 * 0.19;
  b_o[103] = t33 * 0.19;
  b_o[104] = t32 * 0.21;
  b_o[105] = o_tmp * 0.19;
  b_o[106] = t32 * 0.126;
  b_o[107] = t33 * 0.126;
  b_o[108] = t25 * 0.081;
  b_o[109] = t26 * 0.081;
  o_tmp = t2 * t8;
  b_o[110] = o_tmp * t89;
  b_o[111] = o_tmp * t90;
  b_o[112] = -t208;
  b_o[113] = t23;
  b_o[114] = -t209;
  b_o[115] = t23 + t43;
  b_o[116] = t24 + t42;
  b_o[117] = t22 + t60;
  b_o[118] = t28 + t40;
  b_o[119] = t12 * t40 * 607.0;
  o_tmp = b_o_tmp * t8 * t17;
  b_o[120] = o_tmp * -50.0;
  b_o_tmp = c_o_tmp * t8 * t17;
  b_o[121] = b_o_tmp * -50.0;
  b_o[122] = t24;
  b_o[123] = o_tmp * 100.0;
  b_o[124] = b_o_tmp * 100.0;
  b_o[125] = t101 * 1900.0;
  b_o[126] = t38 * 0.19;
  b_o[127] = t37 * 0.21;
  b_o[128] = t38 * 0.21;
  o_tmp = t7 * t22;
  b_o[129] = o_tmp * 0.045;
  b_o[130] = t25;
  b_o[131] = t38 * 0.126;
  b_o[132] = o_tmp * 0.126;
  b_o[133] = t12 * t60 * 607.0;
  b_o[134] = t110 * 1900.0;
  b_o[135] = t45 * 0.19;
  b_o[136] = t46 * 0.19;
  b_o[137] = t42 * 0.21;
  b_o[138] = t47 * 0.19;
  b_o[139] = t26;
  b_o[140] = t48 * 0.19;
  b_o[141] = t43 * 0.21;
  b_o[142] = t45 * 0.21;
  b_o[143] = t50 * 0.19;
  b_o[144] = t46 * 0.21;
  b_o[145] = t51 * 0.19;
  b_o[146] = t50 * 0.21;
  b_o[147] = t51 * 0.21;
  o_tmp = t13 * t22;
  b_o[148] = o_tmp * 0.045;
  b_o[149] = t27;
  b_o_tmp = t7 * t26;
  b_o[150] = b_o_tmp * 0.045;
  b_o[151] = t45 * 0.126;
  b_o[152] = t46 * 0.126;
  b_o[153] = t47 * 0.126;
  b_o[154] = t48 * 0.126;
  b_o[155] = o_tmp * 0.126;
  b_o[156] = b_o_tmp * 0.126;
  b_o[157] = t54 * 0.19;
  b_o[158] = t56 * 0.19;
  b_o[159] = t28;
  b_o[160] = t57 * 0.19;
  b_o[161] = t53 * 0.21;
  b_o[162] = t58 * 0.19;
  b_o[163] = t59 * 0.19;
  b_o[164] = t54 * 0.21;
  b_o[165] = t60 * 0.19;
  b_o[166] = t56 * 0.21;
  b_o[167] = t60 * 0.21;
  b_o[168] = t54 * 0.126;
  b_o[169] = t56 * 0.126;
  b_o[170] = -t20;
  b_o[171] = t57 * 0.126;
  b_o[172] = t58 * 0.126;
  b_o[173] = t59 * 0.126;
  b_o[174] = b_t90_tmp * t14 * -100.0;
  b_o[175] = b_t89_tmp * t16 * -100.0;
  o_tmp = t63_tmp * t23;
  b_o[176] = -(o_tmp * 607.0);
  b_o[177] = -(t63_tmp * t27 * 607.0);
  b_o[178] = t30;
  b_o_tmp = t10 * t12 * t26;
  b_o[179] = -(b_o_tmp * 607.0);
  c_o_tmp = t7 * t25;
  b_o[180] = -(c_o_tmp * 0.045);
  b_o[181] = -(c_o_tmp * 0.126);
  b_o[182] = t31;
  b_o[183] = t314;
  b_o[184] = t315;
  b_o[185] = t32 * 0.081;
  b_o[186] = t34 * 0.081;
  b_o[187] = t318;
  b_o[188] = t319;
  b_o[189] = t32;
  b_o[190] = t25 * 0.0607;
  b_o[191] = t321;
  b_o[192] = t322;
  b_o[193] = t93 * 0.19;
  b_o[194] = t7 * t38 * 0.045;
  b_o[195] = t93 * 0.126;
  b_o[196] = t33;
  b_o[197] = t95 * 0.19;
  b_o[198] = t98 * 0.19;
  b_o[199] = t7 * t45 * 0.045;
  b_o[200] = t7 * t46 * 0.045;
  b_o[201] = t7 * t47 * 0.045;
  b_o[202] = t7 * t48 * 0.045;
  c_o_tmp = t7 * t12;
  b_o[203] = c_o_tmp * t21 * 0.045;
  t123 = t7 * t50;
  b_o[204] = t123 * 0.045;
  t115_tmp = t7 * t51;
  b_o[205] = t115_tmp * 0.045;
  b_o[206] = t34;
  t123_tmp = t13 * t41;
  b_o[207] = t123_tmp * 0.045;
  t115 = t7 * t52;
  b_o[208] = t115 * 0.045;
  b_o[209] = t95 * 0.126;
  b_o[210] = t98 * 0.126;
  b_o[211] = t123 * 0.126;
  b_o[212] = t115_tmp * 0.126;
  b_o[213] = t123_tmp * 0.126;
  b_o[214] = t115 * 0.126;
  b_o[215] = t100 * 0.19;
  b_o[216] = t101 * 0.19;
  b_o[217] = t3 * t9 * 34.0;
  b_o[218] = t102 * 0.19;
  b_o[219] = t101 * 0.21;
  b_o[220] = t13 * t45 * 0.045;
  b_o[221] = t7 * t54 * 0.045;
  b_o[222] = t7 * t56 * 0.045;
  b_o[223] = t7 * t57 * 0.045;
  b_o[224] = t7 * t58 * 0.045;
  b_o[225] = t7 * t59 * 0.045;
  t123 = t12 * t13;
  b_o[226] = t123 * t21 * 0.045;
  t115_tmp = t6 * t13;
  b_o[227] = t115_tmp * t24 * 0.045;
  b_o[228] = c_o_tmp * t24 * 0.045;
  t123_tmp = t7 * t60;
  b_o[229] = t123_tmp * 0.045;
  t115 = t13 * t52;
  b_o[230] = t115 * 0.045;
  b_o[231] = t100 * 0.126;
  b_o[232] = t101 * 0.126;
  b_o[233] = t102 * 0.126;
  b_o[234] = t123_tmp * 0.126;
  b_o[235] = t115 * 0.126;
  b_o[236] = t38 * 0.081;
  b_o[237] = t109 * 0.19;
  b_o[238] = t37;
  b_o[239] = t110 * 0.19;
  b_o[240] = t110 * 0.21;
  b_o[241] = t13 * t56 * 0.045;
  b_o[242] = t7 * t32 * 0.045;
  b_o[243] = t13 * t57 * 0.045;
  b_o[244] = t7 * t33 * 0.045;
  b_o[245] = t123 * t23 * 0.045;
  t123_tmp = t13 * t60;
  b_o[246] = t123_tmp * 0.045;
  b_o[247] = t112 * 0.045;
  t115 = t13 * t62;
  b_o[248] = t115 * 0.045;
  t208 = t13 * t63;
  b_o[249] = t208 * 0.045;
  t11 = t13 * t64;
  b_o[250] = t11 * 0.045;
  t90_tmp = t13 * t65;
  b_o[251] = t90_tmp * 0.045;
  b_o[252] = t109 * 0.126;
  b_o[253] = t110 * 0.126;
  b_o[254] = t111 * 0.126;
  b_o[255] = t123_tmp * 0.126;
  b_o[256] = t39;
  b_o[257] = t112 * 0.126;
  b_o[258] = t208 * 0.126;
  b_o[259] = t11 * 0.126;
  b_o[260] = t90_tmp * 0.126;
  b_o[261] = t45 * 0.081;
  b_o[262] = t46 * 0.081;
  b_o[263] = t50 * 0.081;
  b_o[264] = t51 * 0.081;
  b_o[265] = t13 * t33 * 0.045;
  b_o[266] = t123 * t27 * 0.045;
  t123_tmp = t13 * t34;
  b_o[267] = t123_tmp * 0.045;
  b_o[268] = t116 * 0.126;
  b_o[269] = t123_tmp * 0.126;
  b_o[270] = t54 * 0.081;
  b_o[271] = t56 * 0.081;
  b_o[272] = t60 * 0.081;
  b_o[273] = t409;
  b_o[274] = t41;
  b_o[275] = t62 * 0.081;
  b_o[276] = t411;
  b_o[277] = -(t115_tmp * t21 * 0.045);
  b_o[278] = -(t123 * t24 * 0.045);
  t123_tmp = t13 * t61;
  b_o[279] = -(t123_tmp * 0.045);
  t208 = t7 * t34;
  b_o[280] = -(t208 * 0.045);
  b_o[281] = -(t123_tmp * 0.126);
  b_o[282] = -(t115 * 0.126);
  b_o[283] = -(t208 * 0.126);
  b_o[284] = t44;
  b_o[285] = -t409;
  b_o[286] = -t411;
  b_o[287] = t32 * 0.0607;
  b_o[288] = t33 * 0.0607;
  b_o[289] = t34 * 0.0607;
  b_o[290] = t12 * t322;
  b_o[291] = t45;
  b_o[292] = t38 + t101;
  b_o[293] = t49 + t96;
  b_o[294] = t46 + t110;
  b_o[295] = t58 + t100;
  b_o[296] = t62 + t107;
  b_o[297] = t7 * t93 * 0.045;
  b_o[298] = t7 * t95 * 0.045;
  b_o[299] = t7 * t98 * 0.045;
  b_o[300] = t115_tmp * t39 * 0.045;
  b_o[301] = t13 * t95 * 0.045;
  b_o[302] = t7 * t100 * 0.045;
  b_o[303] = t7 * t101 * 0.045;
  b_o[304] = t7 * t102 * 0.045;
  b_o[305] = t123 * t37 * 0.045;
  b_o[306] = t47;
  b_o[307] = t115_tmp * t42 * 0.045;
  b_o[308] = c_o_tmp * t42 * 0.045;
  b_o[309] = t115_tmp * t44 * 0.045;
  b_o[310] = t115_tmp * t48 * 0.045;
  b_o[311] = t115_tmp * t49 * 0.045;
  b_o[312] = t48;
  b_o[313] = t13 * t102 * 0.045;
  b_o[314] = t7 * t109 * 0.045;
  b_o[315] = t7 * t110 * 0.045;
  b_o[316] = t123 * t42 * 0.045;
  b_o[317] = t123 * t43 * 0.045;
  b_o[318] = t115_tmp * t53 * 0.045;
  b_o[319] = c_o_tmp * t53 * 0.045;
  b_o[320] = t115_tmp * t54 * 0.045;
  b_o[321] = t123 * t48 * 0.045;
  b_o[322] = t115_tmp * t59 * 0.045;
  c_o_tmp = t13 * t107;
  b_o[323] = c_o_tmp * 0.045;
  b_o[324] = c_o_tmp * 0.126;
  b_o[325] = t5;
  b_o[326] = t13 * t111 * 0.045;
  b_o[327] = t123 * t53 * 0.045;
  b_o[328] = t123 * t54 * 0.045;
  b_o[329] = t115_tmp * t32 * 0.045;
  b_o[330] = t123 * t59 * 0.045;
  b_o[331] = t101 * 0.081;
  b_o[332] = t107 * 0.081;
  b_o[333] = t13 * t116 * 0.045;
  b_o[334] = t123 * t32 * 0.045;
  b_o[335] = t37 * 0.0607;
  b_o[336] = t39 * 0.0607;
  b_o[337] = t40 * 0.0607;
  b_o[338] = t110 * 0.081;
  b_o[339] = t111 * 0.081;
  b_o[340] = t112 * 0.081;
  b_o[341] = t42 * 0.0607;
  b_o[342] = t43 * 0.0607;
  b_o[343] = t44 * 0.0607;
  b_o[344] = t52;
  b_o[345] = t45 * 0.0607;
  b_o[346] = t48 * 0.0607;
  b_o[347] = t49 * 0.0607;
  b_o[348] = t12 * t314;
  b_o[349] = t6 * t318;
  b_o[350] = t51 * 0.0607;
  b_o[351] = t6 * t319;
  b_o[352] = t527;
  b_o[353] = d_o_tmp * 0.0607;
  b_o[354] = t12 * t315;
  b_o[355] = t53;
  b_o[356] = t52 * 0.0607;
  b_o[357] = t116 * 0.081;
  b_o[358] = t53 * 0.0607;
  b_o[359] = t54 * 0.0607;
  b_o[360] = t55 * 0.0607;
  b_o[361] = t56 * 0.0607;
  b_o[362] = t57 * 0.0607;
  b_o[363] = t59 * 0.0607;
  b_o[364] = t12 * t319;
  b_o[365] = t60 * 0.0607;
  b_o[366] = t54;
  b_o[367] = t6 * t321;
  b_o[368] = t541;
  b_o[369] = t64 * 0.0607;
  b_o[370] = t55;
  b_o[371] = t115_tmp * t45 * -0.045;
  b_o[372] = t6 * t7 * t54 * -0.045;
  b_o[373] = t56;
  b_o[374] = t115_tmp * t56 * -0.045;
  b_o[375] = t57;
  b_o[376] = t115_tmp * t33 * -0.045;
  b_o[377] = t6 * t21 * -0.0607;
  b_o[378] = -t527;
  b_o[379] = t6 * t27 * -0.0607;
  b_o[380] = -t541;
  b_o[381] = t7 * t21 * t41 * 0.045;
  b_o[382] = t23 * 0.4 + t24 * t30;
  b_o[383] = t75 + t23 * t30;
  b_o[384] = t59;
  b_o[385] = t7 * t24 * t41 * 0.045;
  b_o[386] = t123 * t93 * 0.045;
  b_o[387] = t115_tmp * t97 * 0.045;
  b_o[388] = t123 * t98 * 0.045;
  b_o[389] = t6;
  b_o[390] = t115_tmp * t104 * 0.045;
  b_o[391] = t93 * 0.0607;
  b_o[392] = t95 * 0.0607;
  b_o[393] = t96 * 0.0607;
  b_o[394] = t97 * 0.0607;
  b_o[395] = t98 * 0.0607;
  b_o[396] = t102 * 0.0607;
  b_o[397] = t103 * 0.0607;
  b_o[398] = t104 * 0.0607;
  b_o[399] = t63;
  b_o[400] = b_o_tmp * 0.0607;
  b_o[401] = t64;
  b_o[402] = t13 * t23 * t41 * -0.045;
  b_o[403] = t65;
  b_o[404] = t66;
  b_o[405] = o_tmp * -0.0607;
  b_o[406] = t12 * t45 * -0.0607;
  b_o[407] = t6 * t54 * -0.0607;
  b_o[408] = t12 * t49 * -0.0607;
  b_o[409] = t12 * t53 * -0.0607;
  b_o[410] = t67;
  b_o[411] = t68;
  b_o[412] = t2 * t30;
  b_o[413] = t7;
  b_o[414] = t70;
  b_o[415] = t22 * 0.4;
  b_o[416] = t2 * t31;
  b_o[417] = t8 * t30;
  b_o[418] = t76;
  b_o[419] = t26 * 0.4;
  b_o[420] = t8 * t31;
  b_o[421] = t27 * 0.4;
  b_o[422] = t8;
  b_o[423] = -(t15 * 17.0);
  b_o[424] = t22 * 1900.0;
  b_o[425] = -t76;
  b_o[426] = t26 * 1900.0;
  b_o[427] = t8 * t9 * -0.4;
  b_o[428] = t2 * t17 * 17.0;
  b_o[429] = t89;
  b_o[430] = t9;
  b_o[431] = t90;
  b_o[432] = t8 * t17 * 17.0;
  b_o[433] = t93;
  b_o[434] = t6 * t40;
  b_o[435] = t96;
  b_o[436] = t97;
  b_o[437] = t98;
  b_o[438] = t6 * t51;
  st.site = &ac_emlrtRSI;
  ft_1(&st, b_o, D_dn3);
}

/* End of code generation (get_ddndq_3.c) */
