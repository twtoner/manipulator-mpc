/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_collisionFcn_U_info.c
 *
 * Code generation for function 'collisionFcn_U'
 *
 */

/* Include files */
#include "_coder_collisionFcn_U_info.h"
#include "emlrt.h"
#include "tmwtypes.h"

/* Function Declarations */
static const mxArray *emlrtMexFcnResolvedFunctionsInfo(void);

/* Function Definitions */
static const mxArray *emlrtMexFcnResolvedFunctionsInfo(void)
{
  const mxArray *nameCaptureInfo;
  const char_T *data[9] = {
      "789ced98cb6f12411cc707538dc61789b1f19126f5a427c32eb43c6ea53caaec9677d116"
      "c3637780857d50760b052ffd078c3735f1c2a9e95dbd79d09b770ffe"
      "13c678eb55282cdda5d90259332d537ec96698fdcdeee7bbf363bf99596079465b0000b7"
      "402f7ed87aedcdceb1df39acfdf397803e86f3967e7b4dd3dfd78cbf",
      "0ce606d769c7bfedb78c242a7057e975c49c000757b292c08939514934ab10d4a02cf175"
      "c81e650a1c0f139c00e3dace7ab7270434a941a79beafe5e2d41a612"
      "df1140ad241f2be4b59dc17c7cd03c2fd03cefdc98f3411acc4777dc13b03dc86ff95f05"
      "3d69bab9e8ab7175985ef1c7c2f1c8a2d3614b576b5219324a9a9178",
      "9e9339490c306226f954d0ebac9ad479e3149dda7c112a1956ccb0acc86e6bf95993fc2b"
      "86fc5e869576f23c3ce6b54df23c863c7d7e745d76148ee7140eca69"
      "eddc0cd7276ba0f7f6987ab5edbe6efc55f0b373f2f39f5f16943ceb47e20b4a9e1a67c5"
      "db35e08dfb7f9b37e05987f24b85682066135ee4a1bcb1499264dec1",
      "f149efb18e88810e8de45375004d5fab03d5fddb06f73fafefadd9badf19a157cd773424"
      "6a39512e483541ce705c23e7d4e9c89ad431a9bf1e98e4ad18f2f4f9"
      "09eb343c476ab990f9c2bbdf687dd6f63cf21e254f0ddc7d9683767bb9c995a3126cb96a"
      "6187c8b8d78218f92c6eefefb8baaf1aeaee658ea89942c52c6f388c",
      "e6498dffb59f581ec153f35babe3d7459d936e3150bdf79e365a1fddfbfa298a92a706ee"
      "3e1aa114578b112bbc20d109badc70c71cc246101f1f6d1bdcffbcae"
      "57b326f58ef2cf3edb7e56eb5264fe39793dec28fdd38178bf6ffbeeb98f92a706eefe99"
      "6a6c7a05aa2945e3c1408ade08306b255f6be69f67e69f66bfa35e1f",
      "a157cd1f69e80ac8d875fcac493eeaefa86e439e3e3f717d7a73837c7f8fda57170ffe2e"
      "a0e4a931adbe7ad780671dcafbf3be6485a5ca6edacdb89986c23a44"
      "aa0466be8af9ba9498ad4b4fd483c0795dbaf7facd3c4a9e1ad3ea9fe3ae4bd7897a7429"
      "1c14a1b314f51645675c592b7281997f62ee9fe4cc3f4fd483c4d93f",
      "6d87ce7b28796ae0ee9fe15028e26b846ca597f978304e57ed65a21099edeb2fd0be9ed0"
      "f1b326f978edeb09ecf7f560e1f103a4bc7ee0eeab149fac174b519f"
      "33d7f2ae16920a9fe2a1ec9ff9eac5f15552c7cf9ae4e3e5ab24f6befaedf0d143943c35"
      "a6d557c7fd5e1a09b172cab5be4225a898cbbb4c2c39694106d3efab",
      "ff00a00d2dea",
      ""};
  nameCaptureInfo = NULL;
  emlrtNameCaptureMxArrayR2016a(&data[0], 12512U, &nameCaptureInfo);
  return nameCaptureInfo;
}

mxArray *emlrtMexFcnProperties(void)
{
  mxArray *xEntryPoints;
  mxArray *xInputs;
  mxArray *xResult;
  const char_T *epFieldName[6] = {
      "Name",           "NumberOfInputs", "NumberOfOutputs",
      "ConstantInputs", "FullPath",       "TimeStamp"};
  const char_T *propFieldName[5] = {"Version", "ResolvedFunctions",
                                    "EntryPoints", "CoverageInfo",
                                    "IsPolymorphic"};
  xEntryPoints =
      emlrtCreateStructMatrix(1, 1, 6, (const char_T **)&epFieldName[0]);
  xInputs = emlrtCreateLogicalMatrix(1, 6);
  emlrtSetField(xEntryPoints, 0, (const char_T *)"Name",
                emlrtMxCreateString((const char_T *)"collisionFcn_U"));
  emlrtSetField(xEntryPoints, 0, (const char_T *)"NumberOfInputs",
                emlrtMxCreateDoubleScalar(6.0));
  emlrtSetField(xEntryPoints, 0, (const char_T *)"NumberOfOutputs",
                emlrtMxCreateDoubleScalar(4.0));
  emlrtSetField(xEntryPoints, 0, (const char_T *)"ConstantInputs", xInputs);
  emlrtSetField(
      xEntryPoints, 0, (const char_T *)"FullPath",
      emlrtMxCreateString(
          (const char_T
               *)"G:\\My Drive\\AEROSP 740\\project\\collisionFcn_U.m"));
  emlrtSetField(xEntryPoints, 0, (const char_T *)"TimeStamp",
                emlrtMxCreateDoubleScalar(738501.51851851854));
  xResult =
      emlrtCreateStructMatrix(1, 1, 5, (const char_T **)&propFieldName[0]);
  emlrtSetField(xResult, 0, (const char_T *)"Version",
                emlrtMxCreateString((const char_T *)"9.10.0.1602886 (R2021a)"));
  emlrtSetField(xResult, 0, (const char_T *)"ResolvedFunctions",
                (mxArray *)emlrtMexFcnResolvedFunctionsInfo());
  emlrtSetField(xResult, 0, (const char_T *)"EntryPoints", xEntryPoints);
  return xResult;
}

/* End of code generation (_coder_collisionFcn_U_info.c) */
