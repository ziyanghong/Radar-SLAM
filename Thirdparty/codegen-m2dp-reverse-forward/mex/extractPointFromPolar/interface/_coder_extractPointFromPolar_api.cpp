/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_extractPointFromPolar_api.cpp
 *
 * Code generation for function '_coder_extractPointFromPolar_api'
 *
 */

/* Include files */
#include "_coder_extractPointFromPolar_api.h"
#include "extractPointFromPolar.h"
#include "extractPointFromPolar_data.h"
#include "extractPointFromPolar_emxutil.h"
#include "rt_nonfinite.h"

/* Variable Definitions */
static emlrtRTEInfo yd_emlrtRTEI = { 1,/* lineNo */
  1,                                   /* colNo */
  "_coder_extractPointFromPolar_api",  /* fName */
  ""                                   /* pName */
};

/* Function Declarations */
static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *img, const
  char_T *identifier, emxArray_uint8_T *y);
static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, emxArray_uint8_T *y);
static real_T e_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *max_selected_distance, const char_T *identifier);
static const mxArray *emlrt_marshallOut(const emxArray_real_T *u);
static real_T f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static void h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, emxArray_uint8_T *ret);
static real_T i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId);

/* Function Definitions */
static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *img, const
  char_T *identifier, emxArray_uint8_T *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = const_cast<const char *>(identifier);
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  d_emlrt_marshallIn(sp, emlrtAlias(img), &thisId, y);
  emlrtDestroyArray(&img);
}

static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, emxArray_uint8_T *y)
{
  h_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static real_T e_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *max_selected_distance, const char_T *identifier)
{
  real_T y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = const_cast<const char *>(identifier);
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = f_emlrt_marshallIn(sp, emlrtAlias(max_selected_distance), &thisId);
  emlrtDestroyArray(&max_selected_distance);
  return y;
}

static const mxArray *emlrt_marshallOut(const emxArray_real_T *u)
{
  const mxArray *y;
  const mxArray *m;
  static const int32_T iv[2] = { 0, 0 };

  y = NULL;
  m = emlrtCreateNumericArray(2, iv, mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, &u->data[0]);
  emlrtSetDimensions((mxArray *)m, u->size, 2);
  emlrtAssign(&y, m);
  return y;
}

static real_T f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = i_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static void h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, emxArray_uint8_T *ret)
{
  static const int32_T dims[2] = { -1, -1 };

  const boolean_T bv[2] = { true, true };

  int32_T iv[2];
  int32_T i;
  emlrtCheckVsBuiltInR2012b(sp, msgId, src, "uint8", false, 2U, dims, &bv[0], iv);
  ret->allocatedSize = iv[0] * iv[1];
  i = ret->size[0] * ret->size[1];
  ret->size[0] = iv[0];
  ret->size[1] = iv[1];
  emxEnsureCapacity_uint8_T(sp, ret, i, (emlrtRTEInfo *)NULL);
  ret->data = (uint8_T *)emlrtMxGetData(src);
  ret->canFreeData = false;
  emlrtDestroyArray(&src);
}

static real_T i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId)
{
  real_T ret;
  static const int32_T dims = 0;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 0U, &dims);
  ret = *(real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

void extractPointFromPolar_api(extractPointFromPolarStackData *SD, const mxArray
  * const prhs[5], int32_T, const mxArray *plhs[1])
{
  emxArray_uint8_T *img;
  emxArray_real_T *pointCloudForward;
  const mxArray *prhs_copy_idx_0;
  real_T max_selected_distance;
  real_T radar_resolution;
  real_T max_distance;
  real_T num_points_per_beam;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  emlrtHeapReferenceStackEnterFcnR2012b(&st);
  emxInit_uint8_T(&st, &img, 2, &yd_emlrtRTEI, true);
  emxInit_real_T(&st, &pointCloudForward, 2, &yd_emlrtRTEI, true);
  prhs_copy_idx_0 = emlrtProtectR2012b(prhs[0], 0, false, -1);

  /* Marshall function inputs */
  img->canFreeData = false;
  c_emlrt_marshallIn(&st, emlrtAlias(prhs_copy_idx_0), "img", img);
  max_selected_distance = e_emlrt_marshallIn(&st, emlrtAliasP(prhs[1]),
    "max_selected_distance");
  radar_resolution = e_emlrt_marshallIn(&st, emlrtAliasP(prhs[2]),
    "radar_resolution");
  max_distance = e_emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "max_distance");
  num_points_per_beam = e_emlrt_marshallIn(&st, emlrtAliasP(prhs[4]),
    "num_points_per_beam");

  /* Invoke the target function */
  extractPointFromPolar(SD, &st, img, max_selected_distance, radar_resolution,
                        max_distance, num_points_per_beam, pointCloudForward);

  /* Marshall function outputs */
  pointCloudForward->canFreeData = false;
  plhs[0] = emlrt_marshallOut(pointCloudForward);
  emxFree_real_T(&pointCloudForward);
  emxFree_uint8_T(&img);
  emlrtHeapReferenceStackLeaveFcnR2012b(&st);
}

/* End of code generation (_coder_extractPointFromPolar_api.cpp) */
