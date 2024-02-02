/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * scalexpAlloc.cpp
 *
 * Code generation for function 'scalexpAlloc'
 *
 */

/* Include files */
#include "scalexpAlloc.h"
#include "extractPointFromPolar.h"
#include "rt_nonfinite.h"

/* Function Definitions */
boolean_T dimagree(const emxArray_real_T *z, const emxArray_real_T *varargin_1,
                   const emxArray_real_T *varargin_2)
{
  boolean_T p;
  boolean_T b_p;
  int32_T k;
  boolean_T exitg1;
  int32_T b_k;
  int32_T c_k;
  p = true;
  b_p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 2)) {
    if (k + 1 <= 1) {
      b_k = z->size[0];
      c_k = varargin_1->size[0];
    } else {
      b_k = 1;
      c_k = 1;
    }

    if (b_k != c_k) {
      b_p = false;
      exitg1 = true;
    } else {
      k++;
    }
  }

  if (b_p) {
    b_p = true;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k < 2)) {
      if (k + 1 <= 1) {
        b_k = z->size[0];
        c_k = varargin_2->size[0];
      } else {
        b_k = 1;
        c_k = 1;
      }

      if (b_k != c_k) {
        b_p = false;
        exitg1 = true;
      } else {
        k++;
      }
    }

    if (!b_p) {
      p = false;
    }
  } else {
    p = false;
  }

  return p;
}

/* End of code generation (scalexpAlloc.cpp) */
