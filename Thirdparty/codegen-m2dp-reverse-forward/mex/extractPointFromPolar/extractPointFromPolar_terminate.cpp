/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * extractPointFromPolar_terminate.cpp
 *
 * Code generation for function 'extractPointFromPolar_terminate'
 *
 */

/* Include files */
#include "extractPointFromPolar_terminate.h"
#include "_coder_extractPointFromPolar_mex.h"
#include "extractPointFromPolar.h"
#include "extractPointFromPolar_data.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void extractPointFromPolar_atexit()
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

void extractPointFromPolar_terminate()
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/* End of code generation (extractPointFromPolar_terminate.cpp) */
