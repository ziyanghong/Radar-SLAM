/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * std.cpp
 *
 * Code generation for function 'std'
 *
 */

/* Include files */
#include "std.h"
#include "blas.h"
#include "eml_int_forloop_overflow_check.h"
#include "extractPointFromPolar.h"
#include "extractPointFromPolar_data.h"
#include "extractPointFromPolar_emxutil.h"
#include "mwmathutil.h"
#include "rt_nonfinite.h"

/* Variable Definitions */
static emlrtRSInfo fe_emlrtRSI = { 9,  /* lineNo */
  "std",                               /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/datafun/std.m"/* pathName */
};

static emlrtRSInfo ge_emlrtRSI = { 100,/* lineNo */
  "varstd",                            /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/datafun/private/varstd.m"/* pathName */
};

static emlrtRSInfo he_emlrtRSI = { 94, /* lineNo */
  "vvarstd",                           /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/datafun/private/vvarstd.m"/* pathName */
};

static emlrtRSInfo ie_emlrtRSI = { 125,/* lineNo */
  "vvarstd",                           /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/datafun/private/vvarstd.m"/* pathName */
};

static emlrtRSInfo je_emlrtRSI = { 136,/* lineNo */
  "vvarstd",                           /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/datafun/private/vvarstd.m"/* pathName */
};

static emlrtRSInfo ke_emlrtRSI = { 141,/* lineNo */
  "vvarstd",                           /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/datafun/private/vvarstd.m"/* pathName */
};

static emlrtRSInfo le_emlrtRSI = { 35, /* lineNo */
  "xnrm2",                             /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/+blas/xnrm2.m"/* pathName */
};

static emlrtRTEInfo wd_emlrtRTEI = { 100,/* lineNo */
  9,                                   /* colNo */
  "varstd",                            /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/datafun/private/varstd.m"/* pName */
};

static emlrtRTEInfo xd_emlrtRTEI = { 124,/* lineNo */
  9,                                   /* colNo */
  "vvarstd",                           /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/datafun/private/vvarstd.m"/* pName */
};

/* Function Definitions */
real_T b_std(const emlrtStack *sp, const emxArray_real_T *x)
{
  real_T y;
  int32_T n;
  real_T xbar;
  int32_T k;
  emxArray_real_T *absdiff;
  ptrdiff_t n_t;
  ptrdiff_t incx_t;
  emlrtStack st;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  f_st.prev = &e_st;
  f_st.tls = e_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  st.site = &fe_emlrtRSI;
  n = x->size[1];
  b_st.site = &ge_emlrtRSI;
  if (x->size[1] == 0) {
    y = rtNaN;
  } else if (x->size[1] == 1) {
    if (!muDoubleScalarIsNaN(x->data[0])) {
      y = 0.0;
    } else {
      y = rtNaN;
    }
  } else {
    c_st.site = &he_emlrtRSI;
    d_st.site = &de_emlrtRSI;
    xbar = x->data[0];
    e_st.site = &ee_emlrtRSI;
    if (x->size[1] > 2147483646) {
      f_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&f_st);
    }

    for (k = 2; k <= n; k++) {
      xbar += x->data[k - 1];
    }

    emxInit_real_T(&d_st, &absdiff, 1, &xd_emlrtRTEI, true);
    xbar /= static_cast<real_T>(x->size[1]);
    k = absdiff->size[0];
    absdiff->size[0] = x->size[1];
    emxEnsureCapacity_real_T(&b_st, absdiff, k, &wd_emlrtRTEI);
    c_st.site = &ie_emlrtRSI;
    if (x->size[1] > 2147483646) {
      d_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&d_st);
    }

    for (k = 0; k < n; k++) {
      absdiff->data[k] = muDoubleScalarAbs(x->data[k] - xbar);
    }

    c_st.site = &je_emlrtRSI;
    d_st.site = &le_emlrtRSI;
    n_t = (ptrdiff_t)x->size[1];
    incx_t = (ptrdiff_t)1;
    y = dnrm2(&n_t, &absdiff->data[0], &incx_t);
    c_st.site = &ke_emlrtRSI;
    y /= muDoubleScalarSqrt(static_cast<real_T>(x->size[1]) - 1.0);
    emxFree_real_T(&absdiff);
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
  return y;
}

/* End of code generation (std.cpp) */
