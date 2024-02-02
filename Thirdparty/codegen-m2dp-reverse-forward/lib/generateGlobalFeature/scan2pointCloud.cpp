/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * scan2pointCloud.cpp
 *
 * Code generation for function 'scan2pointCloud'
 *
 */

/* Include files */
#include "scan2pointCloud.h"
#include "findpeaks.h"
#include "generateGlobalFeature.h"
#include "generateGlobalFeature_emxutil.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

/* Function Definitions */
void scan2pointCloud(emxArray_uint8_T *img, double max_selected_distance, double
                     radar_resolution, double max_distance, emxArray_real_T
                     *pointCloudForward, emxArray_real_T *pointCloudOpposite)
{
  emxArray_uint8_T *b_img;
  double max_pixel;
  int loop_ub;
  int b_loop_ub;
  int i;
  int i1;
  emxArray_real_T *t;
  int k;
  static double b_pointCloudForward[30000];
  int counter;
  emxArray_real_T *pks;
  emxArray_real_T *locs;
  emxArray_real_T *c_img;
  int b_i;
  double peaks_mean;
  double d;
  double peaks_std;
  double scale;
  double b_t;
  emxInit_uint8_T(&b_img, 2);

  /*  input: */
  /*  img: polar image */
  /*  max_selected_distance: maximum distance chosen by user in meters */
  /*  radar_resolution: m / pixel */
  /*  max_distance: maximum distance for each azimuth sacn in meters */
  max_pixel = std::floor(max_selected_distance / max_distance * static_cast<
    double>(img->size[1]));
  if (1.0 > max_pixel) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(max_pixel);
  }

  b_loop_ub = img->size[0] - 1;
  i = b_img->size[0] * b_img->size[1];
  b_img->size[0] = img->size[0];
  b_img->size[1] = loop_ub;
  emxEnsureCapacity_uint8_T(b_img, i);
  for (i = 0; i < loop_ub; i++) {
    for (i1 = 0; i1 <= b_loop_ub; i1++) {
      b_img->data[i1 + b_img->size[0] * i] = img->data[i1 + img->size[0] * i];
    }
  }

  i = img->size[0] * img->size[1];
  img->size[0] = b_img->size[0];
  img->size[1] = b_img->size[1];
  emxEnsureCapacity_uint8_T(img, i);
  loop_ub = b_img->size[0] * b_img->size[1];
  for (i = 0; i < loop_ub; i++) {
    img->data[i] = b_img->data[i];
  }

  emxFree_uint8_T(&b_img);
  emxInit_real_T(&t, 2);
  if (img->size[1] < 1) {
    t->size[0] = 1;
    t->size[1] = 0;
  } else {
    i = t->size[0] * t->size[1];
    t->size[0] = 1;
    t->size[1] = static_cast<int>((static_cast<double>(img->size[1]) - 1.0)) + 1;
    emxEnsureCapacity_real_T(t, i);
    loop_ub = static_cast<int>((static_cast<double>(img->size[1]) - 1.0));
    for (i = 0; i <= loop_ub; i++) {
      t->data[i] = static_cast<double>(i) + 1.0;
    }
  }

  loop_ub = img->size[0];
  if (max_pixel > 600.0) {
    b_loop_ub = 60;
  } else {
    b_loop_ub = 30;
  }

  for (i = 0; i < b_loop_ub; i++) {
    for (i1 = 0; i1 < loop_ub; i1++) {
      img->data[i1 + img->size[0] * i] = 0U;
    }
  }

  if (max_pixel > img->size[1]) {
    i = 0;
    i1 = 0;
  } else {
    i = static_cast<int>(max_pixel) - 1;
    i1 = img->size[1];
  }

  loop_ub = img->size[0];
  b_loop_ub = i1 - i;
  for (i1 = 0; i1 < b_loop_ub; i1++) {
    for (k = 0; k < loop_ub; k++) {
      img->data[k + img->size[0] * (i + i1)] = 0U;
    }
  }

  /*  allocated_memoery = 2000; % defalut */
  std::memset(&b_pointCloudForward[0], 0, 30000U * sizeof(double));

  /*  tic */
  counter = 0;
  i = img->size[0];
  emxInit_real_T(&pks, 2);
  emxInit_real_T(&locs, 2);
  emxInit_real_T(&c_img, 2);
  for (b_i = 0; b_i < i; b_i++) {
    /*      tic */
    loop_ub = img->size[1];
    i1 = c_img->size[0] * c_img->size[1];
    c_img->size[0] = 1;
    c_img->size[1] = img->size[1];
    emxEnsureCapacity_real_T(c_img, i1);
    for (i1 = 0; i1 < loop_ub; i1++) {
      c_img->data[i1] = img->data[b_i + img->size[0] * i1];
    }

    findpeaks(c_img, t, pks, locs);

    /*      toc */
    i1 = pks->size[1];
    if (pks->size[1] == 0) {
      max_pixel = 0.0;
    } else {
      max_pixel = pks->data[0];
      for (k = 2; k <= i1; k++) {
        max_pixel += pks->data[k - 1];
      }
    }

    b_loop_ub = pks->size[1];
    peaks_mean = max_pixel / static_cast<double>(pks->size[1]);
    if (pks->size[1] == 0) {
      peaks_std = rtNaN;
    } else if (pks->size[1] == 1) {
      if ((!rtIsInf(pks->data[0])) && (!rtIsNaN(pks->data[0]))) {
        peaks_std = 0.0;
      } else {
        peaks_std = rtNaN;
      }
    } else {
      max_pixel = pks->data[0];
      for (k = 2; k <= b_loop_ub; k++) {
        max_pixel += pks->data[k - 1];
      }

      max_pixel /= static_cast<double>(pks->size[1]);
      peaks_std = 0.0;
      scale = 3.3121686421112381E-170;
      for (k = 0; k < b_loop_ub; k++) {
        d = std::abs(pks->data[k] - max_pixel);
        if (d > scale) {
          b_t = scale / d;
          peaks_std = peaks_std * b_t * b_t + 1.0;
          scale = d;
        } else {
          b_t = d / scale;
          peaks_std += b_t * b_t;
        }
      }

      peaks_std = scale * std::sqrt(peaks_std);
      peaks_std /= std::sqrt(static_cast<double>(pks->size[1]) - 1.0);
    }

    i1 = pks->size[1];
    for (b_loop_ub = 0; b_loop_ub < i1; b_loop_ub++) {
      if (pks->data[b_loop_ub] > peaks_mean + peaks_std) {
        max_pixel = 1.5707963267948966 - (static_cast<double>(b_i) + 1.0) * 2.0 *
          3.1415926535897931 / 400.0;

        /*  Forward view */
        scale = locs->data[b_loop_ub] * radar_resolution;
        b_pointCloudForward[counter] = scale * std::cos(max_pixel);
        b_pointCloudForward[counter + 10000] = scale * std::sin(max_pixel);
        b_pointCloudForward[counter + 20000] = 1.0;
        counter++;
      }
    }
  }

  emxFree_real_T(&c_img);
  emxFree_real_T(&locs);
  emxFree_real_T(&pks);
  emxFree_real_T(&t);
  i = pointCloudForward->size[0] * pointCloudForward->size[1];
  pointCloudForward->size[0] = counter + 1;
  pointCloudForward->size[1] = 3;
  emxEnsureCapacity_real_T(pointCloudForward, i);
  i = pointCloudOpposite->size[0] * pointCloudOpposite->size[1];
  pointCloudOpposite->size[0] = counter + 1;
  pointCloudOpposite->size[1] = 3;
  emxEnsureCapacity_real_T(pointCloudOpposite, i);
  for (i = 0; i < 3; i++) {
    for (i1 = 0; i1 <= counter; i1++) {
      d = b_pointCloudForward[i1 + 10000 * i];
      pointCloudForward->data[i1 + pointCloudForward->size[0] * i] = d;
      pointCloudOpposite->data[i1 + pointCloudOpposite->size[0] * i] = d;
    }
  }

  for (i = 0; i <= counter; i++) {
    pointCloudOpposite->data[i] = -b_pointCloudForward[i];
  }

  for (i = 0; i <= counter; i++) {
    pointCloudOpposite->data[i + pointCloudOpposite->size[0]] =
      -b_pointCloudForward[i + 10000];
  }

  /*  toc */
}

/* End of code generation (scan2pointCloud.cpp) */
