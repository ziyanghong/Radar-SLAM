/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * xrot.h
 *
 * Code generation for function 'xrot'
 *
 */

#ifndef XROT_H
#define XROT_H

/* Include files */
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "generateGlobalFeature_types.h"

/* Function Declarations */
extern void b_xrot(int n, emxArray_real_T *x, int ix0, int iy0, double c, double
                   s);
extern void c_xrot(double x[16384], int ix0, int iy0, double c, double s);
extern void d_xrot(double x[4096], int ix0, int iy0, double c, double s);
extern void xrot(double x[9], int ix0, int iy0, double c, double s);

#endif

/* End of code generation (xrot.h) */
