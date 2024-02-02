/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * extractPointFromPolar_emxutil.h
 *
 * Code generation for function 'extractPointFromPolar_emxutil'
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
CODEGEN_EXPORT_SYM void emxEnsureCapacity_boolean_T(const emlrtStack *sp,
  emxArray_boolean_T *emxArray, int32_T oldNumel, const emlrtRTEInfo
  *srcLocation);
CODEGEN_EXPORT_SYM void emxEnsureCapacity_int32_T(const emlrtStack *sp,
  emxArray_int32_T *emxArray, int32_T oldNumel, const emlrtRTEInfo *srcLocation);
CODEGEN_EXPORT_SYM void emxEnsureCapacity_real_T(const emlrtStack *sp,
  emxArray_real_T *emxArray, int32_T oldNumel, const emlrtRTEInfo *srcLocation);
CODEGEN_EXPORT_SYM void emxEnsureCapacity_uint8_T(const emlrtStack *sp,
  emxArray_uint8_T *emxArray, int32_T oldNumel, const emlrtRTEInfo *srcLocation);
CODEGEN_EXPORT_SYM void emxFree_boolean_T(emxArray_boolean_T **pEmxArray);
CODEGEN_EXPORT_SYM void emxFree_int32_T(emxArray_int32_T **pEmxArray);
CODEGEN_EXPORT_SYM void emxFree_real_T(emxArray_real_T **pEmxArray);
CODEGEN_EXPORT_SYM void emxFree_uint8_T(emxArray_uint8_T **pEmxArray);
CODEGEN_EXPORT_SYM void emxInit_boolean_T(const emlrtStack *sp,
  emxArray_boolean_T **pEmxArray, int32_T numDimensions, const emlrtRTEInfo
  *srcLocation, boolean_T doPush);
CODEGEN_EXPORT_SYM void emxInit_int32_T(const emlrtStack *sp, emxArray_int32_T **
  pEmxArray, int32_T numDimensions, const emlrtRTEInfo *srcLocation, boolean_T
  doPush);
CODEGEN_EXPORT_SYM void emxInit_real_T(const emlrtStack *sp, emxArray_real_T
  **pEmxArray, int32_T numDimensions, const emlrtRTEInfo *srcLocation, boolean_T
  doPush);
CODEGEN_EXPORT_SYM void emxInit_uint8_T(const emlrtStack *sp, emxArray_uint8_T **
  pEmxArray, int32_T numDimensions, const emlrtRTEInfo *srcLocation, boolean_T
  doPush);

/* End of code generation (extractPointFromPolar_emxutil.h) */
