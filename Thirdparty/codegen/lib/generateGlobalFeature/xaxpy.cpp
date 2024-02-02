/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * xaxpy.cpp
 *
 * Code generation for function 'xaxpy'
 *
 */

/* Include files */
#include "xaxpy.h"
#include "generateGlobalFeature.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void b_xaxpy(int n, double a, const emxArray_real_T *x, int ix0, emxArray_real_T
             *y, int iy0)
{
  int ix;
  int iy;
  int i;
  int k;
  if ((n >= 1) && (!(a == 0.0))) {
    ix = ix0 - 1;
    iy = iy0 - 1;
    i = n - 1;
    for (k = 0; k <= i; k++) {
      y->data[iy] += a * x->data[ix];
      ix++;
      iy++;
    }
  }
}

void c_xaxpy(double a, double y[9], int iy0)
{
  int iy;
  if (!(a == 0.0)) {
    iy = iy0 - 1;
    y[iy] += a * y[1];
    iy++;
    y[iy] += a * y[2];
  }
}

void d_xaxpy(int n, double a, int ix0, double y[8192], int iy0)
{
  int ix;
  int iy;
  int i;
  int k;
  if (!(a == 0.0)) {
    ix = ix0 - 1;
    iy = iy0 - 1;
    i = n - 1;
    for (k = 0; k <= i; k++) {
      y[iy] += a * y[ix];
      ix++;
      iy++;
    }
  }
}

void e_xaxpy(int n, double a, const double x[8192], int ix0, double y[64], int
             iy0)
{
  int ix;
  int iy;
  int i;
  int k;
  if ((n >= 1) && (!(a == 0.0))) {
    ix = ix0 - 1;
    iy = iy0 - 1;
    i = n - 1;
    for (k = 0; k <= i; k++) {
      y[iy] += a * x[ix];
      ix++;
      iy++;
    }
  }
}

void f_xaxpy(int n, double a, const double x[64], int ix0, double y[8192], int
             iy0)
{
  int ix;
  int iy;
  int i;
  int k;
  if ((n >= 1) && (!(a == 0.0))) {
    ix = ix0 - 1;
    iy = iy0 - 1;
    i = n - 1;
    for (k = 0; k <= i; k++) {
      y[iy] += a * x[ix];
      ix++;
      iy++;
    }
  }
}

void g_xaxpy(int n, double a, int ix0, double y[4096], int iy0)
{
  int ix;
  int iy;
  int i;
  int k;
  if (!(a == 0.0)) {
    ix = ix0 - 1;
    iy = iy0 - 1;
    i = n - 1;
    for (k = 0; k <= i; k++) {
      y[iy] += a * y[ix];
      ix++;
      iy++;
    }
  }
}

void h_xaxpy(int n, double a, int ix0, double y[16384], int iy0)
{
  int ix;
  int iy;
  int i;
  int k;
  if (!(a == 0.0)) {
    ix = ix0 - 1;
    iy = iy0 - 1;
    i = n - 1;
    for (k = 0; k <= i; k++) {
      y[iy] += a * y[ix];
      ix++;
      iy++;
    }
  }
}

void xaxpy(int n, double a, int ix0, emxArray_real_T *y, int iy0)
{
  int ix;
  int iy;
  int i;
  int k;
  if ((n >= 1) && (!(a == 0.0))) {
    ix = ix0 - 1;
    iy = iy0 - 1;
    i = n - 1;
    for (k = 0; k <= i; k++) {
      y->data[iy] += a * y->data[ix];
      ix++;
      iy++;
    }
  }
}

/* End of code generation (xaxpy.cpp) */
