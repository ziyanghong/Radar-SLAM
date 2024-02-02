/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * eml_setop.h
 *
 * Code generation for function 'eml_setop'
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
void b_do_vectors(const emlrtStack *sp, const emxArray_int32_T *a, const
                  emxArray_int32_T *b, emxArray_int32_T *c, emxArray_int32_T *ia,
                  emxArray_int32_T *ib);
void do_vectors(const emlrtStack *sp, const emxArray_int32_T *a, const
                emxArray_int32_T *b, emxArray_int32_T *c, emxArray_int32_T *ia,
                emxArray_int32_T *ib);

/* End of code generation (eml_setop.h) */
