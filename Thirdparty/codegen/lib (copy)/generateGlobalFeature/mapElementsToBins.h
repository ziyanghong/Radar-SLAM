/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * mapElementsToBins.h
 *
 * Code generation for function 'mapElementsToBins'
 *
 */

#ifndef MAPELEMENTSTOBINS_H
#define MAPELEMENTSTOBINS_H

/* Include files */
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "generateGlobalFeature_types.h"

/* Function Declarations */
extern void b_mapElementsToBins(const emxArray_real_T *x, const double edges[9],
  int n[8], int bin_data[], int bin_size[2]);
extern void mapElementsToBins(const emxArray_real_T *x, const double edges[17],
  int n[16], int bin_data[], int bin_size[2]);

#endif

/* End of code generation (mapElementsToBins.h) */
