/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * extractPointFromPolar_data.cpp
 *
 * Code generation for function 'extractPointFromPolar_data'
 *
 */

/* Include files */
#include "extractPointFromPolar_data.h"
#include "extractPointFromPolar.h"
#include "rt_nonfinite.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;
const volatile char_T *emlrtBreakCheckR2012bFlagVar = NULL;
emlrtContext emlrtContextGlobal = { true,/* bFirstTime */
  false,                               /* bInitialized */
  131483U,                             /* fVersionInfo */
  NULL,                                /* fErrorFunction */
  "extractPointFromPolar",             /* fFunctionName */
  NULL,                                /* fRTCallStack */
  false,                               /* bDebugMode */
  { 2045744189U, 2170104910U, 2743257031U, 4284093946U },/* fSigWrd */
  NULL                                 /* fSigMem */
};

emlrtRSInfo u_emlrtRSI = { 21,         /* lineNo */
  "eml_int_forloop_overflow_check",    /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"/* pathName */
};

emlrtRSInfo gb_emlrtRSI = { 23,        /* lineNo */
  "flipud",                            /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/elmat/flipud.m"/* pathName */
};

emlrtRSInfo tc_emlrtRSI = { 118,       /* lineNo */
  "eml_integer_colon_dispatcher",      /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/ops/colon.m"/* pathName */
};

emlrtRSInfo fd_emlrtRSI = { 50,        /* lineNo */
  "prodsize",                          /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/shared/coder/coder/+coder/+internal/prodsize.m"/* pathName */
};

emlrtRSInfo pd_emlrtRSI = { 587,       /* lineNo */
  "merge_pow2_block",                  /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/sortIdx.m"/* pathName */
};

emlrtRSInfo qd_emlrtRSI = { 589,       /* lineNo */
  "merge_pow2_block",                  /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/sortIdx.m"/* pathName */
};

emlrtRSInfo rd_emlrtRSI = { 617,       /* lineNo */
  "merge_pow2_block",                  /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/sortIdx.m"/* pathName */
};

emlrtRSInfo td_emlrtRSI = { 506,       /* lineNo */
  "merge_block",                       /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/sortIdx.m"/* pathName */
};

emlrtRSInfo de_emlrtRSI = { 124,       /* lineNo */
  "combineVectorElements",             /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/datafun/private/combineVectorElements.m"/* pathName */
};

emlrtRSInfo ee_emlrtRSI = { 184,       /* lineNo */
  "colMajorFlatIter",                  /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/datafun/private/combineVectorElements.m"/* pathName */
};

emlrtRTEInfo emlrtRTEI = { 18,         /* lineNo */
  15,                                  /* colNo */
  "mean",                              /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/datafun/mean.m"/* pName */
};

emlrtRTEInfo o_emlrtRTEI = { 47,       /* lineNo */
  27,                                  /* colNo */
  "varstd",                            /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/datafun/private/varstd.m"/* pName */
};

/* End of code generation (extractPointFromPolar_data.cpp) */
