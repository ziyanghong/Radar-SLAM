/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_extractPointFromPolar_mex.cpp
 *
 * Code generation for function '_coder_extractPointFromPolar_mex'
 *
 */

/* Include files */
#include "_coder_extractPointFromPolar_mex.h"
#include "_coder_extractPointFromPolar_api.h"
#include "extractPointFromPolar.h"
#include "extractPointFromPolar_data.h"
#include "extractPointFromPolar_initialize.h"
#include "extractPointFromPolar_terminate.h"

/* Function Declarations */
MEXFUNCTION_LINKAGE void c_extractPointFromPolar_mexFunc
  (extractPointFromPolarStackData *SD, int32_T nlhs, mxArray *plhs[1], int32_T
   nrhs, const mxArray *prhs[5]);

/* Function Definitions */
void c_extractPointFromPolar_mexFunc(extractPointFromPolarStackData *SD, int32_T
  nlhs, mxArray *plhs[1], int32_T nrhs, const mxArray *prhs[5])
{
  const mxArray *outputs[1];
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 5) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 5, 4,
                        21, "extractPointFromPolar");
  }

  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 21,
                        "extractPointFromPolar");
  }

  /* Call the function. */
  extractPointFromPolar_api(SD, prhs, nlhs, outputs);

  /* Copy over outputs to the caller. */
  emlrtReturnArrays(1, plhs, outputs);
}

void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs, const mxArray
                 *prhs[])
{
  extractPointFromPolarStackData *c_extractPointFromPolarStackDat = NULL;
  c_extractPointFromPolarStackDat = (extractPointFromPolarStackData *)
    emlrtMxCalloc(1, (size_t)1U * sizeof(extractPointFromPolarStackData));
  mexAtExit(extractPointFromPolar_atexit);

  /* Module initialization. */
  extractPointFromPolar_initialize();

  /* Dispatch the entry-point. */
  c_extractPointFromPolar_mexFunc(c_extractPointFromPolarStackDat, nlhs, plhs,
    nrhs, prhs);

  /* Module termination. */
  extractPointFromPolar_terminate();
  emlrtMxFree(c_extractPointFromPolarStackDat);
}

emlrtCTX mexFunctionCreateRootTLS()
{
  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  return emlrtRootTLSGlobal;
}

/* End of code generation (_coder_extractPointFromPolar_mex.cpp) */
