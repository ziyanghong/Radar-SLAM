/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * flipud.cpp
 *
 * Code generation for function 'flipud'
 *
 */

/* Include files */
#include "flipud.h"
#include "extractPointFromPolar.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void flipud(emxArray_int32_T *x)
{
  int32_T m;
  int32_T md2;
  int32_T i;
  int32_T xtmp;
  int32_T b_i;
  m = x->size[0] - 1;
  md2 = x->size[0] >> 1;
  for (i = 0; i < md2; i++) {
    xtmp = x->data[i];
    b_i = m - i;
    x->data[i] = x->data[b_i];
    x->data[b_i] = xtmp;
  }
}

/* End of code generation (flipud.cpp) */
