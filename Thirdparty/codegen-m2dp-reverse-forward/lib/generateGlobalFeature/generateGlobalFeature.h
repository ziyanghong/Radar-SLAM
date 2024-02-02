/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * generateGlobalFeature.h
 *
 * Code generation for function 'generateGlobalFeature'
 *
 */

#ifndef GENERATEGLOBALFEATURE_H
#define GENERATEGLOBALFEATURE_H

/* Include files */
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "generateGlobalFeature_types.h"

/* Function Declarations */
extern void generateGlobalFeature(const emxArray_uint8_T *img, double
  max_selected_distance, double range_resolution, double max_distance, double
  featureForward[192], emxArray_real_T *pointCloudForward, double
  featureReverse[192], emxArray_real_T *pointCloudOpposite);

#endif

/* End of code generation (generateGlobalFeature.h) */
