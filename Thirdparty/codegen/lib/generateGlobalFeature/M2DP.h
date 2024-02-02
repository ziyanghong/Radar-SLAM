/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * M2DP.h
 *
 * Code generation for function 'M2DP'
 *
 */

#ifndef M2DP_H
#define M2DP_H

/* Include files */
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "generateGlobalFeature_types.h"

/* Function Declarations */
extern void GetSignatureMatrix(const emxArray_real_T *data, double maxRho,
  double A[8192]);
extern void PCARotationInvariant(emxArray_real_T *data);

#endif

/* End of code generation (M2DP.h) */
