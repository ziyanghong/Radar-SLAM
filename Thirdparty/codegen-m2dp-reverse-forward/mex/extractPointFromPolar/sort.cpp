/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sort.cpp
 *
 * Code generation for function 'sort'
 *
 */

/* Include files */
#include "sort.h"
#include "eml_int_forloop_overflow_check.h"
#include "extractPointFromPolar.h"
#include "extractPointFromPolar_data.h"
#include "extractPointFromPolar_emxutil.h"
#include "rt_nonfinite.h"
#include "sortIdx.h"

/* Variable Definitions */
static emlrtRSInfo yc_emlrtRSI = { 76, /* lineNo */
  "sort",                              /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/sort.m"/* pathName */
};

static emlrtRSInfo ad_emlrtRSI = { 79, /* lineNo */
  "sort",                              /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/sort.m"/* pathName */
};

static emlrtRSInfo bd_emlrtRSI = { 81, /* lineNo */
  "sort",                              /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/sort.m"/* pathName */
};

static emlrtRSInfo cd_emlrtRSI = { 84, /* lineNo */
  "sort",                              /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/sort.m"/* pathName */
};

static emlrtRSInfo dd_emlrtRSI = { 87, /* lineNo */
  "sort",                              /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/sort.m"/* pathName */
};

static emlrtRSInfo ed_emlrtRSI = { 90, /* lineNo */
  "sort",                              /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/sort.m"/* pathName */
};

static emlrtRTEInfo df_emlrtRTEI = { 1,/* lineNo */
  20,                                  /* colNo */
  "sort",                              /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/sort.m"/* pName */
};

static emlrtRTEInfo ef_emlrtRTEI = { 56,/* lineNo */
  1,                                   /* colNo */
  "sort",                              /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/sort.m"/* pName */
};

/* Function Definitions */
void sort(const emlrtStack *sp, emxArray_int32_T *x)
{
  int32_T dim;
  emxArray_int32_T *vwork;
  int32_T i;
  int32_T vlen;
  int32_T j;
  int32_T vstride;
  int32_T k;
  emxArray_int32_T *ne_emlrtRSI;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  dim = 0;
  if (x->size[0] != 1) {
    dim = -1;
  }

  emxInit_int32_T(sp, &vwork, 1, &ef_emlrtRTEI, true);
  if (dim + 2 <= 1) {
    i = x->size[0];
  } else {
    i = 1;
  }

  vlen = i - 1;
  j = vwork->size[0];
  vwork->size[0] = i;
  emxEnsureCapacity_int32_T(sp, vwork, j, &df_emlrtRTEI);
  st.site = &yc_emlrtRSI;
  vstride = 1;
  for (k = 0; k <= dim; k++) {
    vstride *= x->size[0];
  }

  st.site = &ad_emlrtRSI;
  st.site = &bd_emlrtRSI;
  if ((1 <= vstride) && (vstride > 2147483646)) {
    b_st.site = &u_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }

  emxInit_int32_T(sp, &ne_emlrtRSI, 1, &df_emlrtRTEI, true);
  for (j = 0; j < vstride; j++) {
    st.site = &cd_emlrtRSI;
    if ((1 <= i) && (i > 2147483646)) {
      b_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    for (k = 0; k <= vlen; k++) {
      vwork->data[k] = x->data[j + k * vstride];
    }

    st.site = &dd_emlrtRSI;
    sortIdx(&st, vwork, ne_emlrtRSI);
    st.site = &ed_emlrtRSI;
    for (k = 0; k <= vlen; k++) {
      x->data[j + k * vstride] = vwork->data[k];
    }
  }

  emxFree_int32_T(&ne_emlrtRSI);
  emxFree_int32_T(&vwork);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (sort.cpp) */
