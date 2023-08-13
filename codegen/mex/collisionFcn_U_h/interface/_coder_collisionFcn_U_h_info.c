/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_collisionFcn_U_h_info.c
 *
 * Code generation for function 'collisionFcn_U_h'
 *
 */

/* Include files */
#include "_coder_collisionFcn_U_h_info.h"
#include "emlrt.h"
#include "tmwtypes.h"

/* Function Declarations */
static const mxArray *emlrtMexFcnResolvedFunctionsInfo(void);

/* Function Definitions */
static const mxArray *emlrtMexFcnResolvedFunctionsInfo(void)
{
  const mxArray *nameCaptureInfo;
  const char_T *data[7] = {
      "789ced585d6fd250183e9869969829372eeab204afbc3314d83a7637c6c70c2d1f2b4854"
      "0c1fed8196b5a7480b8c79e31f30de7ae915f167e86ff0c27f61bcdb"
      "ad83b670d852e9d259c3b16f420e2fcf29cfc37bca93270581e76c0000700f18f52964ac"
      "1b17afef1740d0fcfc1658ac093e06733c60ae77b07e8cedbf0dd666",
      "d7e1fb3f9a2baf221d9eea46831a0a9c5d29a88a841a482f8dba10f4a0a6ca03284c9196"
      "24c392a4400e6f72934e4963d0ac994093f78722e44fb8be027aa236"
      "5728e3cd7c1ed8ef05d8ef5d73388f98cd3c82666fe1af536f32fb5576144af6a401ac1e"
      "a48ef35c2144c7c2d56e4fed405eaff2aa2c4b9aa4a2348f6ae59af8",
      "4cc175d65dea5cb7d569206da8d704548bde149fdd7d12341141ed37657873e7b06bcbb7"
      "882f3f87be2ec9922e41ad6acd647212cbe672dfa14e7c1d2fec5f9f"
      "fe1f633f7f4cb778c517feb6ffc84b3eabfe15dfa90d9fd3fb6cd3862f7809af0c5f2514"
      "66a416b94cbac2be48f34762f22c33d751b0d18149fea30e80f5b80e",
      "afbeff6ffa268ebbf5cdae4b9d7797e8b4f0a9570848785b8b029cbfee92ffba3efad925"
      "5fdc966f11bfb68f1ab3310e0790eba7a12fbfb6bde4b36a55fdf481"
      "0d5ff0129e6a26cb2702d389b3713ece0f7521861811f87eeab59fd65dea749843293f87"
      "5ec9a114c939f4fdbb0f9b5ef259b5aabee93487e6a84171279f4190",
      "168b8936a239fda82da57ddf243f875200e7afbbe4272b8752c4e750b0fdf4b1a77c6691"
      "eea78c5c1eb4c562926e9c250e5b655daec8504bf97e4a680e8df839"
      "f44a0e8d909c43c3e7f4432ff9ac22dd37f3d96c2139cc86c5974d2ec3b1dd68876a15fc"
      "e7a1ff410e8d009cbfee929fac1c1a213e877e3d7fb2e5259f55abea",
      "a74e9f8716b28256d9cb1d3025e6782fb14bedd0aca281d5f7d3df4cde1571",
      ""};
  nameCaptureInfo = NULL;
  emlrtNameCaptureMxArrayR2016a(&data[0], 8368U, &nameCaptureInfo);
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
  xInputs = emlrtCreateLogicalMatrix(1, 7);
  emlrtSetField(xEntryPoints, 0, (const char_T *)"Name",
                emlrtMxCreateString((const char_T *)"collisionFcn_U_h"));
  emlrtSetField(xEntryPoints, 0, (const char_T *)"NumberOfInputs",
                emlrtMxCreateDoubleScalar(7.0));
  emlrtSetField(xEntryPoints, 0, (const char_T *)"NumberOfOutputs",
                emlrtMxCreateDoubleScalar(4.0));
  emlrtSetField(xEntryPoints, 0, (const char_T *)"ConstantInputs", xInputs);
  emlrtSetField(
      xEntryPoints, 0, (const char_T *)"FullPath",
      emlrtMxCreateString(
          (const char_T
               *)"G:\\My Drive\\AEROSP 740\\project\\collisionFcn_U_h.m"));
  emlrtSetField(xEntryPoints, 0, (const char_T *)"TimeStamp",
                emlrtMxCreateDoubleScalar(738501.50545138889));
  xResult =
      emlrtCreateStructMatrix(1, 1, 5, (const char_T **)&propFieldName[0]);
  emlrtSetField(xResult, 0, (const char_T *)"Version",
                emlrtMxCreateString((const char_T *)"9.10.0.1602886 (R2021a)"));
  emlrtSetField(xResult, 0, (const char_T *)"ResolvedFunctions",
                (mxArray *)emlrtMexFcnResolvedFunctionsInfo());
  emlrtSetField(xResult, 0, (const char_T *)"EntryPoints", xEntryPoints);
  return xResult;
}

/* End of code generation (_coder_collisionFcn_U_h_info.c) */
