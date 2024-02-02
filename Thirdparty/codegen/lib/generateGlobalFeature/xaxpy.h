/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * xaxpy.h
 *
 * Code generation for function 'xaxpy'
 *
 */

#ifndef XAXPY_H
#define XAXPY_H

/* Include files */
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "generateGlobalFeature_types.h"

/* Function Declarations */
extern void b_xaxpy(int n, double a, const emxArray_real_T *x, int ix0,
                    emxArray_real_T *y, int iy0);
extern void c_xaxpy(double a, double y[9], int iy0);
extern void d_xaxpy(int n, double a, int ix0, double y[8192], int iy0);
extern void e_xaxpy(int n, double a, const double x[8192], int ix0, double y[64],
                    int iy0);
extern void f_xaxpy(int n, double a, const double x[64], int ix0, double y[8192],
                    int iy0);
extern void g_xaxpy(int n, double a, int ix0, double y[4096], int iy0);
extern void h_xaxpy(int n, double a, int ix0, double y[16384], int iy0);
extern void xaxpy(int n, double a, int ix0, emxArray_real_T *y, int iy0);

#endif

/* End of code generation (xaxpy.h) */
