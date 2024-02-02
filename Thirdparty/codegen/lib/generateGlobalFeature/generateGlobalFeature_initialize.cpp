/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * generateGlobalFeature_initialize.cpp
 *
 * Code generation for function 'generateGlobalFeature_initialize'
 *
 */

/* Include files */
#include "generateGlobalFeature_initialize.h"
#include "generateGlobalFeature.h"
#include "generateGlobalFeature_data.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void generateGlobalFeature_initialize()
{
  rt_InitInfAndNaN();
  isInitialized_generateGlobalFeature = true;
}

/* End of code generation (generateGlobalFeature_initialize.cpp) */
