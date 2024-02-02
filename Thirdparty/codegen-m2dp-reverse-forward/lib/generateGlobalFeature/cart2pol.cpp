/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * cart2pol.cpp
 *
 * Code generation for function 'cart2pol'
 *
 */

/* Include files */
#include "cart2pol.h"
#include "generateGlobalFeature.h"
#include "generateGlobalFeature_emxutil.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <math.h>

/* Function Declarations */
static double rt_atan2d_snf(double u0, double u1);
static double rt_hypotd_snf(double u0, double u1);

/* Function Definitions */
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  int b_u0;
  int b_u1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }

    if (u1 > 0.0) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }

    y = atan2(static_cast<double>(b_u0), static_cast<double>(b_u1));
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

static double rt_hypotd_snf(double u0, double u1)
{
  double y;
  double a;
  a = std::abs(u0);
  y = std::abs(u1);
  if (a < y) {
    a /= y;
    y *= std::sqrt(a * a + 1.0);
  } else if (a > y) {
    y /= a;
    y = a * std::sqrt(y * y + 1.0);
  } else {
    if (!rtIsNaN(y)) {
      y = a * 1.4142135623730951;
    }
  }

  return y;
}

void cart2pol(const emxArray_real_T *x, const emxArray_real_T *y,
              emxArray_real_T *th, emxArray_real_T *r)
{
  int nx;
  int k;
  nx = th->size[0];
  if (y->size[0] <= x->size[0]) {
    th->size[0] = static_cast<short>(y->size[0]);
  } else {
    th->size[0] = static_cast<short>(x->size[0]);
  }

  emxEnsureCapacity_real_T(th, nx);
  if (y->size[0] <= x->size[0]) {
    nx = static_cast<short>(y->size[0]);
  } else {
    nx = static_cast<short>(x->size[0]);
  }

  for (k = 0; k < nx; k++) {
    th->data[k] = rt_atan2d_snf(y->data[k], x->data[k]);
  }

  nx = r->size[0];
  if (x->size[0] <= y->size[0]) {
    r->size[0] = static_cast<short>(x->size[0]);
  } else {
    r->size[0] = static_cast<short>(y->size[0]);
  }

  emxEnsureCapacity_real_T(r, nx);
  if (x->size[0] <= y->size[0]) {
    nx = static_cast<short>(x->size[0]);
  } else {
    nx = static_cast<short>(y->size[0]);
  }

  for (k = 0; k < nx; k++) {
    r->data[k] = rt_hypotd_snf(x->data[k], y->data[k]);
  }
}

/* End of code generation (cart2pol.cpp) */
