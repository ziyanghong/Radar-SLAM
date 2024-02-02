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
#include "svd.h"
#include <cmath>
#include <cstring>

/* Function Definitions */
void generateGlobalFeature(const emxArray_uint8_T *img, double
  max_selected_distance, double range_resolution, double max_distance, double
  feature[192], emxArray_real_T *point_cloud_forward)
{
  emxArray_uint8_T *b_img;
  int i;
  int nx;
  emxArray_real_T *unusedU0;
  emxArray_real_T *y;
  int vstride;
  emxArray_real_T *rho2;
  int n;
  double maxRho;
  boolean_T exitg1;
  static double unusedU1[8192];
  double d;
  double u[4096];
  double s[8192];
  static double v[16384];
  if (isInitialized_generateGlobalFeature == false) {
    generateGlobalFeature_initialize();
  }

  emxInit_uint8_T(&b_img, 2);

  /*  Pre-process the polar image */
  i = b_img->size[0] * b_img->size[1];
  b_img->size[0] = img->size[0];
  b_img->size[1] = img->size[1];
  emxEnsureCapacity_uint8_T(b_img, i);
  nx = img->size[0] * img->size[1] - 1;
  for (i = 0; i <= nx; i++) {
    b_img->data[i] = img->data[i];
  }

  emxInit_real_T(&unusedU0, 2);
  scan2pointCloud(b_img, max_selected_distance, range_resolution, max_distance,
                  point_cloud_forward, unusedU0);

  /*  Compute forward descriptor */
  /*  Multiview 2D projection (M2DP) descriptor.  */
  /*  */
  /*  Input: */
  /*        data        n*3     Point cloud. Each row is [x y z] */
  /*  Output: */
  /*        desM2DP     192*1   M2DP descriptor of the input cloud data */
  /*        A           64*128  Signature matrix */
  /*  */
  /*  Introduction: */
  /*  M2DP is a global descriptor of input point cloud. Details of M2DP can be  */
  /*  found in the following paper: */
  /*  */
  /*  Li He, Xiaolong Wang and Hong Zhang, M2DP: A Novel 3D Point Cloud  */
  /*  Descriptor and Its Application in Loop Closure Detection, IROS 2016. */
  /*  */
  /*  Li He, Dept. of Computing Science, University of Alberta */
  /*  lhe2@ualberta.ca */
  /*  1. Initialization */
  /*  key parameter */
  /*  number of bins in theta, the 't' in paper */
  /*  number of bins in rho, the 'l' in paper */
  /*  number of azimuth angles, the 'p' in paper */
  /*  numP = 1; */
  /*  number of elevation angles, the 'q' in paper */
  /*  numQ = 1; */
  /*  rotation invariant */
  i = unusedU0->size[0] * unusedU0->size[1];
  unusedU0->size[0] = point_cloud_forward->size[0];
  unusedU0->size[1] = 3;
  emxEnsureCapacity_real_T(unusedU0, i);
  nx = point_cloud_forward->size[0] * point_cloud_forward->size[1];
  emxFree_uint8_T(&b_img);
  for (i = 0; i < nx; i++) {
    unusedU0->data[i] = point_cloud_forward->data[i];
  }

  emxInit_real_T(&y, 2);
  PCARotationInvariant(unusedU0);

  /*  Azimuthe list */
  /*  Elevation list */
  /*  get the farthest point distance */
  i = y->size[0] * y->size[1];
  y->size[0] = static_cast<short>(unusedU0->size[0]);
  y->size[1] = 3;
  emxEnsureCapacity_real_T(y, i);
  nx = static_cast<short>(unusedU0->size[0]) * 3;
  for (vstride = 0; vstride < nx; vstride++) {
    y->data[vstride] = unusedU0->data[vstride] * unusedU0->data[vstride];
  }

  emxInit_real_T(&rho2, 1);
  vstride = y->size[0];
  i = rho2->size[0];
  rho2->size[0] = static_cast<short>(y->size[0]);
  emxEnsureCapacity_real_T(rho2, i);
  nx = 2 * y->size[0];
  for (n = 0; n < vstride; n++) {
    rho2->data[n] = y->data[n];
    rho2->data[n] += y->data[vstride + n];
    rho2->data[n] += y->data[nx + n];
  }

  emxFree_real_T(&y);
  n = rho2->size[0];
  if (rho2->size[0] <= 2) {
    if (rho2->size[0] == 1) {
      maxRho = rho2->data[0];
    } else if ((rho2->data[0] < rho2->data[1]) || (rtIsNaN(rho2->data[0]) &&
                (!rtIsNaN(rho2->data[1])))) {
      maxRho = rho2->data[1];
    } else {
      maxRho = rho2->data[0];
    }
  } else {
    if (!rtIsNaN(rho2->data[0])) {
      nx = 1;
    } else {
      nx = 0;
      vstride = 2;
      exitg1 = false;
      while ((!exitg1) && (vstride <= rho2->size[0])) {
        if (!rtIsNaN(rho2->data[vstride - 1])) {
          nx = vstride;
          exitg1 = true;
        } else {
          vstride++;
        }
      }
    }

    if (nx == 0) {
      maxRho = rho2->data[0];
    } else {
      maxRho = rho2->data[nx - 1];
      i = nx + 1;
      for (vstride = i; vstride <= n; vstride++) {
        d = rho2->data[vstride - 1];
        if (maxRho < d) {
          maxRho = d;
        }
      }
    }
  }

  emxFree_real_T(&rho2);
  maxRho = std::sqrt(maxRho);

  /*  main function, get the signature matrix A */
  GetSignatureMatrix(unusedU0, maxRho, unusedU1);

  /*  run SVD on A and use [u1,v1] as the final output */
  svd(unusedU1, u, s, v);
  emxFree_real_T(&unusedU0);
  std::memcpy(&feature[0], &u[0], 64U * sizeof(double));
  std::memcpy(&feature[64], &v[0], 128U * sizeof(double));
}

/* End of code generation (generateGlobalFeature.cpp) */
