/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * extractPointFromPolar.h
 *
 * Code generation for function 'extractPointFromPolar'
 *
 */

#pragma once

/* Include files */
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "mex.h"
#include "emlrt.h"
#include "rtwtypes.h"
#include "extractPointFromPolar_types.h"

/* Function Declarations */
CODEGEN_EXPORT_SYM void extractPointFromPolar(extractPointFromPolarStackData *SD,
  const emlrtStack *sp, emxArray_uint8_T *img, real_T max_selected_distance,
  real_T radar_resolution, real_T max_distance, real_T num_points_per_beam,
  emxArray_real_T *pointCloudForward);

/* End of code generation (extractPointFromPolar.h) */
