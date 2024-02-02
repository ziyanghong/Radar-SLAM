/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_generateGlobalFeature_api.h
 *
 * Code generation for function '_coder_generateGlobalFeature_api'
 *
 */

#ifndef _CODER_GENERATEGLOBALFEATURE_API_H
#define _CODER_GENERATEGLOBALFEATURE_API_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"

/* Type Definitions */
#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T

struct emxArray_real_T
{
  real_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_real_T*/

#ifndef typedef_emxArray_real_T
#define typedef_emxArray_real_T

typedef struct emxArray_real_T emxArray_real_T;

#endif                                 /*typedef_emxArray_real_T*/

#ifndef struct_emxArray_uint8_T
#define struct_emxArray_uint8_T

struct emxArray_uint8_T
{
  uint8_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_uint8_T*/

#ifndef typedef_emxArray_uint8_T
#define typedef_emxArray_uint8_T

typedef struct emxArray_uint8_T emxArray_uint8_T;

#endif                                 /*typedef_emxArray_uint8_T*/

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void generateGlobalFeature(emxArray_uint8_T *img, real_T
  max_selected_distance, real_T range_resolution, real_T max_distance, real_T
  feature[192], emxArray_real_T *point_cloud_forward);
extern void generateGlobalFeature_api(const mxArray * const prhs[4], int32_T
  nlhs, const mxArray *plhs[2]);
extern void generateGlobalFeature_atexit(void);
extern void generateGlobalFeature_initialize(void);
extern void generateGlobalFeature_terminate(void);
extern void generateGlobalFeature_xil_shutdown(void);
extern void generateGlobalFeature_xil_terminate(void);

#endif

/* End of code generation (_coder_generateGlobalFeature_api.h) */
