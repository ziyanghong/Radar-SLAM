/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * svd.cpp
 *
 * Code generation for function 'svd'
 *
 */

/* Include files */
#include "svd.h"
#include "generateGlobalFeature.h"
#include "rt_nonfinite.h"
#include "svd1.h"
#include <cstring>

/* Function Definitions */
void svd(const double A[8192], double U[4096], double S[8192], double V[16384])
{
  boolean_T p;
  int i;
  double s[64];
  p = true;
  for (i = 0; i < 8192; i++) {
    if ((!p) || (rtIsInf(A[i]) || rtIsNaN(A[i]))) {
      p = false;
    }
  }

  if (p) {
    b_svd(A, U, s, V);
  } else {
    for (i = 0; i < 4096; i++) {
      U[i] = rtNaN;
    }

    for (i = 0; i < 64; i++) {
      s[i] = rtNaN;
    }

    for (i = 0; i < 16384; i++) {
      V[i] = rtNaN;
    }
  }

  std::memset(&S[0], 0, 8192U * sizeof(double));
  for (i = 0; i < 64; i++) {
    S[i + (i << 6)] = s[i];
  }
}

/* End of code generation (svd.cpp) */
