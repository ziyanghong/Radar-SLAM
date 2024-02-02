/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * generateGlobalFeature.cpp
 *
 * Code generation for function 'generateGlobalFeature'
 *
 */

/* Include files */
#include "generateGlobalFeature.h"
#include "M2DP.h"
#include "generateGlobalFeature_data.h"
#include "generateGlobalFeature_emxutil.h"
#include "generateGlobalFeature_initialize.h"
#include "rt_nonfinite.h"
#include "scan2pointCloud.h"

/* Function Definitions */
void generateGlobalFeature(const emxArray_uint8_T *img, double
  max_selected_distance, double range_resolution, double max_distance, double
  featureForward[192], emxArray_real_T *pointCloudForward, double
  featureReverse[192], emxArray_real_T *pointCloudOpposite)
{
  emxArray_uint8_T *b_img;
  int i;
  int loop_ub;
  emxArray_real_T *b_pointCloudForward;
  emxArray_real_T *b_pointCloudOpposite;
  static double unusedU0[8192];
  if (isInitialized_generateGlobalFeature == false) {
    generateGlobalFeature_initialize();
  }

  emxInit_uint8_T(&b_img, 2);

  /*  Pre-process the polar image */
  i = b_img->size[0] * b_img->size[1];
  b_img->size[0] = img->size[0];
  b_img->size[1] = img->size[1];
  emxEnsureCapacity_uint8_T(b_img, i);
  loop_ub = img->size[0] * img->size[1] - 1;
  for (i = 0; i <= loop_ub; i++) {
    b_img->data[i] = img->data[i];
  }

  emxInit_real_T(&b_pointCloudForward, 2);
  emxInit_real_T(&b_pointCloudOpposite, 2);
  scan2pointCloud(b_img, max_selected_distance, range_resolution, max_distance,
                  b_pointCloudForward, b_pointCloudOpposite);
  i = pointCloudForward->size[0] * pointCloudForward->size[1];
  pointCloudForward->size[0] = b_pointCloudForward->size[0];
  pointCloudForward->size[1] = 3;
  emxEnsureCapacity_real_T(pointCloudForward, i);
  loop_ub = b_pointCloudForward->size[0] * b_pointCloudForward->size[1];
  emxFree_uint8_T(&b_img);
  for (i = 0; i < loop_ub; i++) {
    pointCloudForward->data[i] = b_pointCloudForward->data[i];
  }

  i = pointCloudOpposite->size[0] * pointCloudOpposite->size[1];
  pointCloudOpposite->size[0] = b_pointCloudOpposite->size[0];
  pointCloudOpposite->size[1] = 3;
  emxEnsureCapacity_real_T(pointCloudOpposite, i);
  loop_ub = b_pointCloudOpposite->size[0] * b_pointCloudOpposite->size[1];
  for (i = 0; i < loop_ub; i++) {
    pointCloudOpposite->data[i] = b_pointCloudOpposite->data[i];
  }

  /*  Compute forward descriptor */
  M2DP(b_pointCloudForward, featureForward, unusedU0);

  /*  Compute forward descriptor */
  M2DP(b_pointCloudOpposite, featureReverse, unusedU0);
  emxFree_real_T(&b_pointCloudOpposite);
  emxFree_real_T(&b_pointCloudForward);
}

/* End of code generation (generateGlobalFeature.cpp) */
