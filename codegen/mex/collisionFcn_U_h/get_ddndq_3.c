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
#include "rt_nonfinite.h"
#include "mwmathutil.h"

/* Function Declarations */
static void ft_2(const real_T ct[315], real_T D_dn3[49]);

/* Function Definitions */
static void ft_2(const real_T ct[315], real_T D_dn3[49])
{
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
  t926 = 1.0 / muDoubleScalarSqrt((ct[6] + ct[303]) + ct[7]);
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
  t1378 =
      1.0 / muDoubleScalarSqrt((t1037 * t1037 + t1044 * t1044) + t926 * t926);
  t1396 =
      1.0 / muDoubleScalarSqrt((t1380 * t1380 + t1384 * t1384) + t1389 * t1389);
  t1394 =
      1.0 / muDoubleScalarSqrt((t1358 * t1358 + t1370 * t1370) + t1375 * t1375);
  t1392 =
      1.0 / muDoubleScalarSqrt((t1354 * t1354 + t1365 * t1365) + t1363 * t1363);
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

void get_ddndq_3(const real_T q[7], const real_T o[3], real_T D_dn3[49])
{
  real_T ct[315];
  real_T dv[49];
  real_T b_ct_tmp;
  real_T b_t1163_tmp;
  real_T b_t89_tmp;
  real_T b_t992_tmp;
  real_T b_t993_tmp;
  real_T c_ct_tmp;
  real_T c_t992_tmp;
  real_T c_t993_tmp;
  real_T ct_idx_115;
  real_T ct_idx_116;
  real_T ct_idx_117;
  real_T ct_idx_164;
  real_T ct_idx_166;
  real_T ct_idx_202;
  real_T ct_idx_207;
  real_T ct_idx_207_tmp;
  real_T ct_idx_225;
  real_T ct_idx_230;
  real_T ct_idx_230_tmp;
  real_T ct_idx_24;
  real_T ct_idx_242;
  real_T ct_idx_243;
  real_T ct_idx_24_tmp;
  real_T ct_idx_270;
  real_T ct_idx_287;
  real_T ct_idx_289;
  real_T ct_idx_290;
  real_T ct_idx_293;
  real_T ct_idx_335;
  real_T ct_idx_336;
  real_T ct_idx_337;
  real_T ct_idx_341;
  real_T ct_idx_342;
  real_T ct_idx_343;
  real_T ct_idx_345;
  real_T ct_idx_346;
  real_T ct_idx_349;
  real_T ct_idx_353;
  real_T ct_idx_353_tmp;
  real_T ct_idx_354;
  real_T ct_idx_358;
  real_T ct_idx_359;
  real_T ct_idx_360;
  real_T ct_idx_361;
  real_T ct_idx_362;
  real_T ct_idx_363;
  real_T ct_idx_365;
  real_T ct_idx_369;
  real_T ct_idx_379;
  real_T ct_idx_382;
  real_T ct_idx_391;
  real_T ct_idx_395;
  real_T ct_idx_400;
  real_T ct_idx_400_tmp;
  real_T ct_idx_405;
  real_T ct_idx_405_tmp;
  real_T ct_idx_412;
  real_T ct_idx_415;
  real_T ct_idx_417;
  real_T ct_idx_419;
  real_T ct_idx_42;
  real_T ct_idx_43;
  real_T ct_idx_434;
  real_T ct_idx_438;
  real_T ct_idx_45;
  real_T ct_idx_46;
  real_T ct_idx_47;
  real_T ct_idx_5;
  real_T ct_idx_6;
  real_T ct_tmp;
  real_T d_ct_tmp;
  real_T e_ct_tmp;
  real_T f_ct_tmp;
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
  real_T t1163;
  real_T t1163_tmp;
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
  real_T t323;
  real_T t324;
  real_T t325;
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
  real_T t412;
  real_T t413;
  real_T t415;
  real_T t416;
  real_T t417;
  real_T t418;
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
  real_T t545;
  real_T t546;
  real_T t547;
  real_T t55;
  real_T t557;
  real_T t558;
  real_T t559;
  real_T t56;
  real_T t57;
  real_T t58;
  real_T t583;
  real_T t584;
  real_T t585;
  real_T t59;
  real_T t6;
  real_T t60;
  real_T t61;
  real_T t611;
  real_T t62;
  real_T t625;
  real_T t63;
  real_T t634;
  real_T t638;
  real_T t63_tmp;
  real_T t64;
  real_T t65;
  real_T t656;
  real_T t66;
  real_T t67;
  real_T t670;
  real_T t68;
  real_T t689;
  real_T t690;
  real_T t691;
  real_T t7;
  real_T t70;
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
  real_T t75;
  real_T t750;
  real_T t751;
  real_T t752;
  real_T t756;
  real_T t76;
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
  real_T t8;
  real_T t801;
  real_T t821;
  real_T t835;
  real_T t841;
  real_T t843;
  real_T t855;
  real_T t856;
  real_T t858;
  real_T t866;
  real_T t89;
  real_T t89_tmp;
  real_T t9;
  real_T t90;
  real_T t905;
  real_T t906;
  real_T t90_tmp;
  real_T t93;
  real_T t930;
  real_T t931;
  real_T t95;
  real_T t96;
  real_T t97;
  real_T t98;
  real_T t987;
  real_T t992;
  real_T t992_tmp;
  real_T t993;
  real_T t993_tmp;
  real_T t998;
  real_T t999;
  int32_T i;
  int32_T i1;
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
  t987 = o[1] * t3;
  t90_tmp = t987 * t9;
  t90 = t90_tmp * 100.0;
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
  t835 = t9 * t12;
  t111 = t835 * t23;
  t112 = t7 * t10 * t28;
  t116 = t835 * t27;
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
  ct_idx_5 = t12 * t51;
  ct_idx_6 = t6 * t60;
  ct_idx_24_tmp = t10 * t28;
  ct_idx_24 = ct_idx_24_tmp * 0.4;
  ct_idx_42 = t50 * 0.4;
  ct_idx_43 = t51 * 0.4;
  ct_idx_45 = t54 * 0.4;
  ct_idx_46 = t56 * 0.4;
  ct_idx_47 = t60 * 0.4;
  ct_idx_115 = t23 + t43;
  ct_idx_116 = t24 + t42;
  ct_idx_117 = t22 + t60;
  ct_idx_164 = t54 * 0.21;
  ct_idx_166 = t56 * 0.21;
  ct_idx_202 = t7 * t48 * 0.045;
  ct_idx_207_tmp = t13 * t41;
  ct_idx_207 = ct_idx_207_tmp * 0.045;
  ct_idx_225 = t7 * t59 * 0.045;
  ct_idx_230_tmp = t13 * t52;
  ct_idx_230 = ct_idx_230_tmp * 0.045;
  ct_idx_242 = t7 * t32 * 0.045;
  ct_idx_243 = t13 * t57 * 0.045;
  ct_idx_270 = t54 * 0.081;
  ct_idx_287 = t32 * 0.0607;
  ct_idx_289 = t34 * 0.0607;
  ct_idx_290 = t12 * t322;
  ct_idx_293 = t49 + t96;
  ct_idx_335 = t37 * 0.0607;
  ct_idx_336 = t39 * 0.0607;
  ct_idx_337 = t40 * 0.0607;
  ct_idx_341 = t42 * 0.0607;
  ct_idx_342 = t43 * 0.0607;
  ct_idx_343 = t44 * 0.0607;
  ct_idx_345 = t45 * 0.0607;
  ct_idx_346 = t48 * 0.0607;
  ct_idx_349 = t6 * t318;
  ct_idx_353_tmp = t4 * t6 * t9;
  ct_idx_353 = ct_idx_353_tmp * 0.0607;
  ct_idx_354 = t12 * t315;
  ct_idx_358 = t53 * 0.0607;
  ct_idx_359 = t54 * 0.0607;
  ct_idx_360 = t55 * 0.0607;
  ct_idx_361 = t56 * 0.0607;
  ct_idx_362 = t57 * 0.0607;
  ct_idx_363 = t59 * 0.0607;
  ct_idx_365 = t60 * 0.0607;
  ct_idx_369 = t64 * 0.0607;
  ct_idx_379 = t6 * t27 * -0.0607;
  ct_idx_382 = t23 * 0.4 + t24 * t30;
  ct_idx_391 = t93 * 0.0607;
  ct_idx_395 = t98 * 0.0607;
  ct_idx_400_tmp = t10 * t12 * t26;
  ct_idx_400 = ct_idx_400_tmp * 0.0607;
  ct_idx_405_tmp = t63_tmp * t23;
  ct_idx_405 = ct_idx_405_tmp * -0.0607;
  ct_idx_412 = t2 * t30;
  ct_idx_415 = t22 * 0.4;
  ct_idx_417 = t8 * t30;
  ct_idx_419 = t26 * 0.4;
  ct_idx_434 = t6 * t40;
  ct_idx_438 = t6 * t51;
  t670 = ct_idx_379 * t9;
  t718 = (t32 + t48) + t98;
  t992_tmp = o[1] * t2;
  b_t992_tmp = t992_tmp * t8 * t17;
  c_t992_tmp = o[2] * t2;
  t992 = (((t115 + t66) + c_t992_tmp * t3 * t9 * -50.0) + t209) +
         b_t992_tmp * -50.0;
  t993_tmp = o[0] * t2;
  b_t993_tmp = o[2] * t3;
  c_t993_tmp = t993_tmp * t8 * t17;
  t993 = (((t123 + t67) + b_t993_tmp * t8 * t9 * -50.0) + t208) +
         c_t993_tmp * -50.0;
  t323 = -t53 + t21;
  t324 = -t37 + t27;
  t325 = -t51 + t25;
  t412 = ct_idx_115 * t5;
  t413 = ct_idx_116 * t6;
  t415 = t11 * ct_idx_115;
  t416 = t12 * ct_idx_115;
  t417 = t12 * ct_idx_116;
  t418 = t13 * ct_idx_117;
  t583 = -t97 + t44;
  t584 = -t103 + t39;
  t585 = -t104 + t55;
  t611 = t12 * ct_idx_337;
  t625 = ct_idx_5 * 0.0607;
  t634 = t12 * ct_idx_365;
  t638 = ct_idx_293 * t6;
  t701 = t11 * ct_idx_382;
  t702 = t11 * (t75 + t23 * t30);
  t723 = (t28 + t40) * t6 * 0.081;
  t731 = t23 * 0.21 + t43 * 0.21;
  t733 = t22 * 0.21 + t60 * 0.21;
  t740 = (t54 + t93) + -t59;
  t742 = (t33 + t45) + -t95;
  t744 = (t102 + t57) + -t56;
  t767 = t6 * t718;
  t769 = t12 * t718;
  t785 = t38 * 0.21 + t101 * 0.21;
  t787 = t46 * 0.21 + t110 * 0.21;
  t799 = t22 * 0.081 + t60 * 0.081;
  t835 = t319 + ct_idx_341;
  t4 = ((((-t20 + t68) + -(t15 * 17.0)) + t993_tmp * t3 * t9 * 50.0) +
        t987 * t8 * t9 * 50.0) +
       17.0;
  t987 = (ct_idx_287 + ct_idx_346) + ct_idx_395;
  t998 = muDoubleScalarAbs(t992);
  t999 = muDoubleScalarAbs(t993);
  t545 = t5 * t324;
  t546 = t6 * t323;
  t547 = t7 * t325;
  t557 = t11 * t324;
  t558 = t12 * t323;
  t559 = t12 * t324;
  t656 = t417 * 607.0;
  t689 = t6 * t583;
  t690 = t6 * t584;
  t691 = t6 * t585;
  t704 = t417 * 0.126;
  t717 = t32 + t412;
  t722 = t413 * 0.081;
  t743 = (t34 + t52) + -ct_idx_438;
  t750 = t27 * 0.21 + -(t37 * 0.21);
  t751 = t25 * 0.21 + -(t51 * 0.21);
  t752 = t26 * 0.21 + -(t50 * 0.21);
  t756 = t6 * t418 * 0.081;
  t764 = t12 * t325 * 0.081;
  t782 = t11 * t731;
  t790 = t6 * t740;
  t792 = t12 * t740;
  t797 = t13 * t742;
  t801 = t13 * t744;
  t821 = t13 * t799;
  t841 = t314 + -ct_idx_358;
  t843 = t25 * 0.0607 + -(t51 * 0.0607);
  t855 = t6 * t835;
  t856 = t12 * t835;
  t858 = t12 * (t322 + ct_idx_337);
  t866 = ct_idx_24_tmp * t733 * 0.42;
  t905 = (t33 * 0.126 + t45 * 0.126) + -(t95 * 0.126);
  t906 = (t57 * 0.126 + -(t56 * 0.126)) + t102 * 0.126;
  t930 = muDoubleScalarAbs(t4);
  t931 = muDoubleScalarSign(t4);
  t4 = (ct_idx_359 + -ct_idx_363) + ct_idx_391;
  t55 = t12 * t987;
  t1163_tmp = t6 * t13;
  b_t1163_tmp = t12 * t13;
  t1163 = ((((((t7 * t56 * 0.045 + b_t1163_tmp * t21 * 0.045) +
               -(t7 * t57 * 0.045)) +
              t1163_tmp * t48 * 0.045) +
             t1163_tmp * t32 * 0.045) +
            -(t7 * t102 * 0.045)) +
           -(b_t1163_tmp * t53 * 0.045)) +
          t24 * ct_idx_207;
  ct[0] = o[1];
  ct[1] = t10;
  ct_tmp = ct_idx_43 + -t76;
  ct[2] = (ct_tmp + ct_idx_354) + t634;
  ct[3] = (((t12 * t28 * 607.0 + t26 * 1900.0) + t527_tmp * 607.0) +
           -(t50 * 1900.0)) +
          t12 * t40 * 607.0;
  ct[4] = t6 * t987;
  ct[5] = t55;
  ct[6] = t998 * t998;
  ct[7] = t999 * t999;
  ct[8] = (((t64 * 607.0 + t22 * 1900.0) + t60 * 1900.0) + -(t63 * 607.0)) +
          -(ct_idx_5 * 607.0);
  ct[9] = t6 * t4;
  ct[10] = t12 * t4;
  ct[11] = t12 * ((t33 * 0.0607 + ct_idx_345) + -(t95 * 0.0607));
  ct[12] = t12 * ((ct_idx_362 + -ct_idx_361) + t102 * 0.0607);
  ct[13] = -t55;
  ct[14] =
      ((((-(b_t993_tmp * t9 * 100.0) + t3 * t9 * 34.0) + t2 * t66) + t8 * t67) +
       -(t993_tmp * t17 * 50.0)) +
      -(o[1] * t8 * t17 * 50.0);
  b_ct_tmp = t13 * t62;
  c_ct_tmp = t13 * t107;
  ct[15] =
      ((ct_idx_24 + t112 * 0.126) + -(b_ct_tmp * 0.126)) + -(c_ct_tmp * 0.126);
  ct[16] = ((-(t25 * 0.19) + t51 * 0.19) + ct_idx_354) + t634;
  d_ct_tmp = t7 * t25;
  e_ct_tmp = t7 * t51;
  f_ct_tmp = ct_idx_6 * t13;
  ct[17] = ((-(d_ct_tmp * 0.045) + e_ct_tmp * 0.045) + ct_idx_207) +
           f_ct_tmp * 0.045;
  ct[18] = ((t38 * 1900.0 + t101 * 1900.0) + -(ct_idx_405_tmp * 607.0)) +
           t12 * t583 * 607.0;
  ct[19] = (t32 * 1900.0 + t412 * 1900.0) + t12 * t744 * 607.0;
  ct[20] = ((t46 * 1900.0 + t110 * 1900.0) + -(t63_tmp * t27 * 607.0)) +
           t12 * t585 * 607.0;
  ct_idx_354 = t54 - t545;
  t585 = t12 * ct_idx_354;
  ct[21] = -t13 * (t722 + t585 * 0.081);
  ct[22] = (-(t11 * ct_idx_116 * 1900.0) + t6 * t324 * 607.0) +
           t12 * ct_idx_293 * 607.0;
  t322 = t7 * t22;
  ct_idx_337 = t7 * t60;
  t993_tmp = t13 * t34;
  b_t993_tmp = t13 * ct_idx_438;
  ct[23] =
      (((t322 * 0.045 + ct_idx_337 * 0.045) + ct_idx_230) + t993_tmp * 0.045) +
      -(b_t993_tmp * 0.045);
  t10 = t13 * t22;
  t4 = t13 * t60;
  t55 = t7 * t52;
  t987 = t7 * t34;
  t835 = t7 * ct_idx_438;
  ct[24] = (((t10 * 0.045 + t4 * 0.045) + -(t55 * 0.045)) + -(t987 * 0.045)) +
           t835 * 0.045;
  t634 = t7 * t26;
  t527_tmp = t13 * t65;
  t40 = t7 * t50;
  t66 = t13 * t61;
  t67 = t13 * ct_idx_434;
  ct[25] =
      (((t634 * 0.045 + t527_tmp * 0.045) + -(t40 * 0.045)) + -(t66 * 0.045)) +
      t67 * 0.045;
  ct[26] = (((t10 * 0.126 + t4 * 0.126) + -(t55 * 0.126)) + -(t987 * 0.126)) +
           t835 * 0.126;
  ct[27] = (((t26 * 0.19 + -(t50 * 0.19)) + ct_idx_290) + t527) + t611;
  t10 = (t31 + ct_idx_419) + -ct_idx_42;
  ct[28] = ((t10 + ct_idx_290) + t527) + t611;
  ct[29] = (((t22 * 0.19 + t60 * 0.19) + ct_idx_369) + -t541) + -t625;
  ct[30] = (((ct_tmp + -(d_ct_tmp * 0.126)) + e_ct_tmp * 0.126) +
            ct_idx_207_tmp * 0.126) +
           f_ct_tmp * 0.126;
  ct[31] = (((t7 * t38 * 0.045 + t7 * t101 * 0.045) + t1163_tmp * t44 * 0.045) +
            t13 * t111 * 0.045) +
           -(t1163_tmp * t97 * 0.045);
  ct[32] = (((t1163_tmp * t24 * 0.045 + t1163_tmp * t42 * 0.045) +
             b_t1163_tmp * t54 * 0.045) +
            -(b_t1163_tmp * t59 * 0.045)) +
           b_t1163_tmp * t93 * 0.045;
  ct[33] = (((t7 * t46 * 0.045 + t7 * t110 * 0.045) + ct_idx_230 * t8) +
            t13 * t116 * 0.045) +
           -(t1163_tmp * t104 * 0.045);
  ct[34] = (((-(t1163_tmp * t21 * 0.045) + t1163_tmp * t53 * 0.045) +
             b_t1163_tmp * t48 * 0.045) +
            b_t1163_tmp * t32 * 0.045) +
           b_t1163_tmp * t98 * 0.045;
  t4 = -o[2] + t30;
  ct_tmp = (t4 + ct_idx_415) + ct_idx_47;
  ct[35] = (((ct_tmp + ct_idx_369) + -t541) + -t625) + 0.34;
  ct[36] = t11;
  ct[37] = (((t38 * 0.19 + t101 * 0.19) + t12 * ct_idx_343) + ct_idx_405) +
           t12 * t97 * -0.0607;
  ct[38] = (((t46 * 0.19 + t110 * 0.19) + t12 * ct_idx_360) + t670) +
           t104 * t12 * -0.0607;
  ct[39] = (((t12 * t314 + ct_idx_346 * t6) + ct_idx_287 * t6) +
            t12 * t53 * -0.0607) +
           t319 * t41;
  ct[40] = (((t12 * t319 + t12 * ct_idx_341) + ct_idx_363 * t6) +
            t6 * t54 * -0.0607) +
           t21 * t41 * -0.0607;
  ct[41] = ((((ct_idx_202 + ct_idx_242) + t7 * t98 * 0.045) + ct_idx_243 * t6) +
            t1163_tmp * t56 * -0.045) +
           t24 * ct_idx_230;
  ct[42] = t112;
  ct[43] = ((((t32 * 0.19 + t48 * 0.19) + t98 * 0.19) + t12 * ct_idx_362) +
            t12 * t56 * -0.0607) +
           t319 * t64;
  ct[44] = ((((t7 * t47 * 0.045 + b_t1163_tmp * t23 * 0.045) +
              b_t1163_tmp * t43 * 0.045) +
             -(t1163_tmp * t39 * 0.045)) +
            -(t7 * t109 * 0.045)) +
           t27 * ct_idx_207;
  ct[45] = ((((t10 + t634 * 0.126) + t527_tmp * 0.126) + -(t40 * 0.126)) +
            -(t66 * 0.126)) +
           t67 * 0.126;
  ct[46] = ((((t7 * t54 * 0.045 + -ct_idx_225) + t7 * t93 * 0.045) +
             t1163_tmp * t45 * -0.045) +
            t1163_tmp * t33 * -0.045) +
           t21 * ct_idx_230;
  ct[47] =
      ((((t54 * 0.19 + -(t59 * 0.19)) + t93 * 0.19) + t12 * t45 * -0.0607) +
       t12 * t33 * -0.0607) +
      t314 * t64;
  ct[48] =
      ((((t7 * t58 * 0.045 + b_t1163_tmp * t27 * 0.045) + t7 * t100 * 0.045) +
        -(b_t1163_tmp * t37 * 0.045)) +
       -(t1163_tmp * t49 * 0.045)) +
      t13 * t23 * t41 * -0.045;
  ct[49] = ((((-(t47 * 0.19) + t109 * 0.19) + ct_idx_349) + ct_idx_342 * t6) +
            t12 * ct_idx_336) +
           t103 * t12 * -0.0607;
  ct[50] = ((((t58 * 0.19 + t100 * 0.19) + ct_idx_379) + ct_idx_335 * t6) +
            t12 * t49 * -0.0607) +
           t12 * t96 * -0.0607;
  ct[51] = (((((ct_tmp + t322 * 0.126) + ct_idx_337 * 0.126) +
              ct_idx_230_tmp * 0.126) +
             t993_tmp * 0.126) +
            -(b_t993_tmp * 0.126)) +
           0.34;
  ct[52] = (t992_tmp * t3 * t9 * 50.0 + -(t89_tmp * t8 * t9 * 50.0)) * t930 *
           t931 * 2.0;
  ct_tmp = t7 * t12;
  ct[53] = ((((((ct_tmp * t21 * 0.045 + ct_idx_243) + -(t13 * t56 * 0.045)) +
               ct_idx_202 * t6) +
              t13 * t102 * 0.045) +
             ct_idx_242 * t6) +
            -(ct_tmp * t53 * 0.045)) +
           t7 * t24 * t41 * 0.045;
  ct[54] = t1163;
  ct[55] =
      ((((((t13 * t45 * 0.045 + ct_tmp * t24 * 0.045) + t13 * t33 * 0.045) +
          ct_tmp * t42 * 0.045) +
         ct_idx_225 * t6) +
        -(t13 * t95 * 0.045)) +
       t6 * t7 * t54 * -0.045) +
      -(t7 * t21 * t41 * 0.045);
  ct[56] = ((((((t7 * t45 * 0.045 + t7 * t33 * 0.045) +
                -(b_t1163_tmp * t24 * 0.045)) +
               t1163_tmp * t54 * 0.045) +
              -(t7 * t95 * 0.045)) +
             -(b_t1163_tmp * t42 * 0.045)) +
            -(t1163_tmp * t59 * 0.045)) +
           t21 * ct_idx_207;
  ct[57] = t1163 * t1163;
  ct[58] = ((((((t33 * 0.19 + t45 * 0.19) + -(t95 * 0.19)) + t6 * t319) +
              ct_idx_341 * t6) +
             t12 * ct_idx_359) +
            t12 * t59 * -0.0607) +
           t12 * ct_idx_391;
  ct[59] = ((((((t56 * 0.19 + -(t57 * 0.19)) + -(t102 * 0.19)) +
               t6 * t21 * -0.0607) +
              ct_idx_358 * t6) +
             t12 * ct_idx_346) +
            t12 * ct_idx_287) +
           t12 * ct_idx_395;
  ct[60] = t12;
  ct_tmp = t998 * muDoubleScalarSign(t992);
  ct[61] = ct_tmp *
           ((((t123_tmp * t20 + -t123) + o[1] * t16 * t17 * 50.0) + -t208) +
            c_t993_tmp * 100.0) *
           2.0;
  d_ct_tmp = t999 * muDoubleScalarSign(t993);
  ct[62] = d_ct_tmp *
           ((((t115_tmp * t20 + -t115) + o[0] * t14 * t17 * 50.0) + -t209) +
            b_t992_tmp * 100.0) *
           2.0;
  e_ct_tmp = t2 * t8;
  ct[63] = ct_tmp *
           ((((((t2 * t17 * 17.0 + t89) + t2 * t15 * -17.0) + t2 * t68) +
              c_t992_tmp * t17 * -50.0) +
             e_ct_tmp * t90) +
            b_t89_tmp * t16 * -100.0) *
           2.0;
  ct[64] = d_ct_tmp *
           ((((((t90 + t8 * t17 * 17.0) + t8 * t15 * -17.0) + t8 * t68) +
              o[2] * t8 * t17 * -50.0) +
             e_ct_tmp * t89) +
            t90_tmp * t14 * -100.0) *
           2.0;
  ct[65] = t32 * 0.4;
  ct[66] = ct_idx_24;
  ct[67] = t13;
  ct[68] = t38 * 0.4;
  ct[69] = t45 * 0.4;
  ct[70] = t46 * 0.4;
  ct[71] = ct_idx_43;
  ct[72] = ct_idx_45;
  ct[73] = ct_idx_46;
  ct[74] = ct_idx_47;
  ct[75] = -o[0];
  ct[76] = -ct_idx_42;
  ct[77] = -ct_idx_43;
  ct[78] = -o[2];
  ct[79] = t45 * 1900.0;
  ct[80] = -ct_idx_45;
  ct[81] = -ct_idx_46;
  ct[82] = t56 * 1900.0;
  ct[83] = t28 * t70;
  ct[84] = t28 * t75;
  ct[85] = -(t54 * 1900.0);
  ct[86] = t45 * 0.21;
  ct[87] = t28;
  ct[88] = ct_idx_164;
  ct[89] = ct_idx_166;
  ct[90] = t30;
  ct[91] = t31;
  ct[92] = -ct_idx_164;
  ct[93] = t111 * 0.126;
  ct[94] = t45 * 0.081;
  ct[95] = t116 * 0.126;
  ct[96] = ct_idx_270;
  ct[97] = t409;
  ct[98] = t411;
  ct[99] = ct_idx_117 * t7;
  ct[100] = t415;
  ct[101] = t418;
  ct[102] = -ct_idx_270;
  ct[103] = ct_idx_289;
  ct[104] = t5;
  ct[105] = t111 * 0.081;
  ct[106] = t112 * 0.081;
  ct[107] = ct_idx_345;
  ct[108] = t527;
  ct[109] = ct_idx_353;
  ct[110] = t116 * 0.081;
  ct[111] = ct_idx_359;
  ct[112] = t54;
  ct[113] = t541;
  ct[114] = t545;
  ct[115] = t546;
  ct[116] = t547;
  ct[117] = t56;
  ct[118] = -t418;
  ct[119] = -t527;
  ct[120] = -ct_idx_359;
  ct[121] = t6;
  ct[122] = ct_idx_349 * t9;
  ct[123] = t6 * t321 * t9;
  ct[124] = ct_idx_400;
  ct[125] = t638;
  ct[126] = (t38 + t101) * t7;
  ct[127] = t13 * (ct_idx_6 + t41);
  ct[128] = (t46 + t110) * t7;
  ct[129] = (t58 + t100) * t7;
  ct[130] = t13 * (t62 + t107);
  ct[131] = -t546;
  ct[132] = -t547;
  ct[133] = ct_idx_115 * t6 * 607.0;
  ct[134] = t413 * 607.0;
  ct[135] = -t559;
  ct[136] = t656;
  ct[137] = ct_idx_405;
  ct[138] = t670;
  ct[139] = -t656;
  ct[140] = -(t415 * 1900.0);
  ct[141] = ct_idx_412;
  ct[142] = t7 * (-t109 + t47);
  ct[143] = t5 * ct_idx_382;
  ct[144] = t413 * 0.126;
  ct[145] = t546 * 607.0;
  ct[146] = t545 * 1900.0;
  ct[147] = t7;
  ct[148] = t701;
  ct[149] = t702;
  ct[150] = t704;
  ct[151] = t558 * 607.0;
  ct[152] = t557 * 1900.0;
  ct[153] = ct_idx_415;
  ct_tmp = t37 * 0.4 - t27 * 0.4;
  ct[154] = -t5 * ct_tmp;
  d_ct_tmp = t53 * 0.4 - t70;
  ct[155] = -t11 * d_ct_tmp;
  ct[156] = -t11 * ct_tmp;
  ct[157] = -(t416 * 0.126);
  ct[158] = -t704;
  ct[159] = -(t11 * t323 * 1900.0);
  ct[160] = t2 * t31;
  ct[161] = t546 * 0.126;
  ct[162] = t722;
  ct[163] = t723;
  ct[164] = -t701;
  ct[165] = -t702;
  ct[166] = t558 * 0.126;
  ct[167] = t559 * 0.126;
  ct[168] = t416 * 0.081;
  ct[169] = ct_idx_417;
  ct[170] = t417 * 0.081;
  ct[171] = t24 * 0.21 + t42 * 0.21;
  ct[172] = t733;
  ct[173] = t5 * ct_tmp;
  ct[174] = -t722;
  ct[175] = t11 * d_ct_tmp;
  ct[176] = (ct_idx_5 + t63) + -t64;
  ct[177] = t638 * 0.126;
  ct[178] = t12 * t584 * 607.0;
  ct[179] = t21 * 0.21 + -(t53 * 0.21);
  ct[180] = t751;
  ct[181] = t752;
  ct_tmp = t11 * t7;
  ct[182] = ct_tmp * ct_idx_116 * 0.081;
  ct[183] = t756;
  ct[184] = t545 * 0.081;
  ct[185] = t546 * 0.081;
  ct[186] = t76;
  ct[187] = t557 * 0.081;
  ct[188] = t558 * 0.081;
  ct[189] = t559 * 0.081;
  ct[190] = t764;
  ct[191] = t45 + t557;
  ct[192] = t7 * t718;
  ct[193] = t769;
  ct[194] = ct_idx_419;
  ct[195] = t5 * t413 * -0.081;
  ct[196] = -t756;
  ct[197] = t733 * t733;
  ct[198] = t689 * 0.126;
  ct[199] = t690 * 0.126;
  ct[200] = t691 * 0.126;
  ct[201] = ct_tmp * t323 * 0.081;
  ct[202] = t5 * t417 * 0.0607;
  ct[203] = t8 * t31;
  ct[204] = t545 * 0.0607;
  ct[205] = t557 * 0.0607;
  ct[206] = t785;
  ct[207] = t787;
  ct[208] = t7 * t740;
  ct[209] = t7 * t742;
  ct[210] = t7 * t743;
  ct[211] = t7 * t744;
  ct[212] = t797;
  ct[213] = t13 * t743;
  ct[214] = t5 * t546 * -0.081;
  ct[215] = t801;
  ct[216] = t5 * t750;
  ct[217] = t11 * t750;
  ct[218] = t689 * 0.081;
  ct[219] = t691 * 0.081;
  ct[220] = t5 * t558 * 0.0607;
  ct[221] = -t782;
  ct[222] = t6 * t797;
  ct[223] = t6 * t801;
  ct[224] = t767 * 607.0;
  ct[225] = t769 * 607.0;
  ct[226] = t7 * (t38 * 0.126 + t101 * 0.126);
  ct[227] = t7 * (t46 * 0.126 + t110 * 0.126);
  ct[228] = t7 * (t58 * 0.126 + t100 * 0.126);
  ct[229] = t111 + t689;
  ct[230] = t7 * t799;
  ct[231] = -t76;
  ct[232] = t31 + t752;
  ct[233] = t821;
  ct[234] = t116 + t691;
  ct[235] = t790 * 607.0;
  ct[236] = t767 * 0.126;
  ct[237] = t792 * 607.0;
  ct[238] = t769 * 0.126;
  ct[239] = t12 * t742 * 607.0;
  ct[240] = t7 * (t47 * 0.126 + -(t109 * 0.126));
  ct[241] = t7 * (t25 * 0.081 + -(t51 * 0.081));
  ct[242] = t7 * (t26 * 0.081 + -(t50 * 0.081));
  ct[243] = -t821;
  ct[244] = t6 * t717 * 0.081;
  ct[245] = t790 * 0.126;
  ct[246] = t12 * t717 * 0.081;
  ct[247] = t792 * 0.126;
  ct[248] = t8 * t9 * -0.4;
  ct[249] = t7 * (t38 * 0.081 + t101 * 0.081);
  ct[250] = t7 * (t46 * 0.081 + t110 * 0.081);
  ct[251] = t6 * (t318 + ct_idx_342);
  ct[252] = t855;
  ct[253] = t856;
  ct[254] = t12 * (t315 + ct_idx_365);
  ct[255] = t858;
  ct[256] = ct_idx_412 + t785;
  ct[257] = (t34 * 607.0 + t52 * 607.0) + -(ct_idx_438 * 607.0);
  ct[258] = (ct_idx_24_tmp * 1900.0 + ct_idx_353_tmp * 607.0) +
            -(ct_idx_400_tmp * 607.0);
  ct[259] = ct_idx_417 + t787;
  ct[260] = t866;
  ct[261] = t6 * t841;
  ct[262] = t6 * (t321 + -ct_idx_335);
  ct[263] = t6 * t843;
  ct[264] = t12 * t841;
  ct[265] = t12 * t843;
  ct[266] = -t866;
  ct[267] = -(t13 * (t62 * 0.081 + t107 * 0.081));
  ct[268] = -t855;
  ct[269] = -t856;
  ct[270] = -t858;
  ct[271] = t1163_tmp * (t56 - t415) * -0.081;
  ct[272] = t6 * ct_idx_354 * -0.081;
  ct[273] = t585 * -0.081;
  ct[274] = t32 * 0.081 + t412 * 0.081;
  ct[275] = t12 * (t49 * 0.0607 + t96 * 0.0607);
  ct[276] = (t4 + t733) + 0.34;
  ct[277] = t12 * (ct_idx_343 + -(t97 * 0.0607));
  ct[278] = t12 * (ct_idx_336 + -(t103 * 0.0607));
  ct[279] = t12 * (ct_idx_360 + -(t104 * 0.0607));
  ct[280] = t32 * 0.21 + t5 * t731;
  ct[281] = t34 * 0.081 + t6 * t325 * 0.081;
  ct[282] = -ct_idx_166 + t782;
  ct[283] = ct_idx_287 + t412 * 0.0607;
  ct_tmp = t56 * 0.081 - t415 * 0.081;
  ct[284] = -t7 * ct_tmp;
  ct[285] = -t13 * (t409 - t723);
  ct[286] = -t13 * ct_tmp;
  ct[287] = -ct_idx_361 + t415 * 0.0607;
  ct[288] = t7 * ((t32 * 0.126 + t48 * 0.126) + t98 * 0.126);
  ct[289] = t930;
  ct[290] = t931;
  ct[291] = ((-(t25 * 1900.0) + t12 * t22 * 607.0) + t51 * 1900.0) +
            t12 * t60 * 607.0;
  ct[292] = t7 * ct_tmp;
  ct[293] = t7 * ((t54 * 0.126 + -(t59 * 0.126)) + t93 * 0.126);
  ct[294] = t413 + t792;
  ct[295] = -t13 * (t411 - t764);
  ct[296] = t7 * t905;
  ct[297] = t558 + t767;
  ct[298] = t7 * t906;
  ct[299] = t13 * t905;
  ct[300] = t13 * t906;
  ct[301] = t13 * (t559 - t638);
  ct[302] = t13 * (t416 - t690);
  ct[303] = t930 * t930;
  ct[304] = (ct_idx_24 + ct_idx_353) + -ct_idx_400;
  ct[305] = t7 * (-t50 + t26) + t13 * ((t65 + ct_idx_434) + -t61);
  ct[306] = (b_ct_tmp * 0.045 + -(t112 * 0.045)) + c_ct_tmp * 0.045;
  ct_tmp = t13 * t63;
  b_ct_tmp = t13 * t64;
  c_ct_tmp = ct_idx_5 * t13;
  ct[307] = (ct_tmp * 0.045 + -(b_ct_tmp * 0.045)) + c_ct_tmp * 0.045;
  ct[308] = (ct_tmp * 0.126 + -(b_ct_tmp * 0.126)) + c_ct_tmp * 0.126;
  ct[309] = (ct_idx_24_tmp * 0.19 + ct_idx_353) + -ct_idx_400;
  ct_tmp = t417 - t790;
  ct[310] = -t13 * ct_tmp;
  ct[311] = t733 * t751 * 2.0;
  ct[312] = t733 * t752 * 2.0;
  ct[313] = t7 * ct_tmp;
  ct[314] = (ct_idx_289 + t52 * 0.0607) + -(ct_idx_438 * 0.0607);
  ft_2(ct, dv);
  for (i = 0; i < 7; i++) {
    for (i1 = 0; i1 < 7; i1++) {
      D_dn3[i1 + 7 * i] = dv[i + 7 * i1];
    }
  }
}

/* End of code generation (get_ddndq_3.c) */
