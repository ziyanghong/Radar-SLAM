/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * findpeaks.h
 *
 * Code generation for function 'findpeaks'
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
void findpeaks(const emlrtStack *sp, const emxArray_real_T *Yin, const
               emxArray_real_T *varargin_1, emxArray_real_T *Ypk,
               emxArray_real_T *Xpk);

/* End of code generation (findpeaks.h) */
