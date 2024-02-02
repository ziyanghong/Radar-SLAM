/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * extractPointFromPolar_initialize.cpp
 *
 * Code generation for function 'extractPointFromPolar_initialize'
 *
 */

/* Include files */
#include "extractPointFromPolar_initialize.h"
#include "_coder_extractPointFromPolar_mex.h"
#include "extractPointFromPolar.h"
#include "extractPointFromPolar_data.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void extractPointFromPolar_initialize()
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  mex_InitInfAndNan();
  mexFunctionCreateRootTLS();
  emlrtBreakCheckR2012bFlagVar = emlrtGetBreakCheckFlagAddressR2012b();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, 0);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/* End of code generation (extractPointFromPolar_initialize.cpp) */
