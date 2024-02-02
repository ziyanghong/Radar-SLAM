/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * repmat.cpp
 *
 * Code generation for function 'repmat'
 *
 */

/* Include files */
#include "repmat.h"
#include "generateGlobalFeature.h"
#include "generateGlobalFeature_emxutil.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void repmat(const double a[3], double varargin_1, emxArray_real_T *b)
{
  int i;
  int jcol;
  int ibmat;
  int itilerow;
  i = static_cast<int>(varargin_1);
  jcol = b->size[0] * b->size[1];
  b->size[0] = static_cast<short>(i);
  b->size[1] = 3;
  emxEnsureCapacity_real_T(b, jcol);
  for (jcol = 0; jcol < 3; jcol++) {
    ibmat = jcol * i;
    for (itilerow = 0; itilerow < i; itilerow++) {
      b->data[ibmat + itilerow] = a[jcol];
    }
  }
}

/* End of code generation (repmat.cpp) */
