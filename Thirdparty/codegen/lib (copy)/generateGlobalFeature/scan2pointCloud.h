/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * scan2pointCloud.h
 *
 * Code generation for function 'scan2pointCloud'
 *
 */

#ifndef SCAN2POINTCLOUD_H
#define SCAN2POINTCLOUD_H

/* Include files */
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "generateGlobalFeature_types.h"

/* Function Declarations */
extern void scan2pointCloud(emxArray_uint8_T *img, double max_selected_distance,
  double radar_resolution, double max_distance, emxArray_real_T
  *pointCloudForward, emxArray_real_T *pointCloudOpposite);

#endif

/* End of code generation (scan2pointCloud.h) */
