/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * main.cpp
 *
 * Code generation for function 'main'
 *
 */

/*************************************************************************/
/* This automatically generated example C++ main file shows how to call  */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/

/* Include files */
#include "main.h"
#include "generateGlobalFeature.h"
#include "generateGlobalFeature_emxAPI.h"
#include "generateGlobalFeature_terminate.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static double argInit_real_T();
static unsigned char argInit_uint8_T();
static emxArray_uint8_T *c_argInit_UnboundedxUnbounded_u();
static void main_generateGlobalFeature();

/* Function Definitions */
static double argInit_real_T()
{
  return 0.0;
}

static unsigned char argInit_uint8_T()
{
  return 0U;
}

static emxArray_uint8_T *c_argInit_UnboundedxUnbounded_u()
{
  emxArray_uint8_T *result;
  int idx0;
  int idx1;

  /* Set the size of the array.
     Change this size to the value that the application requires. */
  result = emxCreate_uint8_T(2, 2);

  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < result->size[0U]; idx0++) {
    for (idx1 = 0; idx1 < result->size[1U]; idx1++) {
      /* Set the value of the array element.
         Change this value to the value that the application requires. */
      result->data[idx0 + result->size[0] * idx1] = argInit_uint8_T();
    }
  }

  return result;
}

static void main_generateGlobalFeature()
{
  emxArray_real_T *point_cloud_forward;
  emxArray_uint8_T *img;
  double max_selected_distance_tmp_tmp;
  double feature[192];
  emxInitArray_real_T(&point_cloud_forward, 2);

  /* Initialize function 'generateGlobalFeature' input arguments. */
  /* Initialize function input argument 'img'. */
  img = c_argInit_UnboundedxUnbounded_u();
  max_selected_distance_tmp_tmp = argInit_real_T();

  /* Call the entry-point 'generateGlobalFeature'. */
  generateGlobalFeature(img, max_selected_distance_tmp_tmp,
                        max_selected_distance_tmp_tmp,
                        max_selected_distance_tmp_tmp, feature,
                        point_cloud_forward);
  emxDestroyArray_real_T(point_cloud_forward);
  emxDestroyArray_uint8_T(img);
}

int main(int, const char * const [])
{
  /* The initialize function is being called automatically from your entry-point function. So, a call to initialize is not included here. */
  /* Invoke the entry-point functions.
     You can call entry-point functions multiple times. */
  main_generateGlobalFeature();

  /* Terminate the application.
     You do not need to do this more than one time. */
  generateGlobalFeature_terminate();
  return 0;
}

/* End of code generation (main.cpp) */
