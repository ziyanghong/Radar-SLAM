/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * issorted.cpp
 *
 * Code generation for function 'issorted'
 *
 */

/* Include files */
#include "issorted.h"
#include "eml_int_forloop_overflow_check.h"
#include "extractPointFromPolar.h"
#include "extractPointFromPolar_data.h"
#include "rt_nonfinite.h"

/* Variable Definitions */
static emlrtRSInfo hc_emlrtRSI = { 71, /* lineNo */
  "issorted",                          /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/datafun/issorted.m"/* pathName */
};

static emlrtRSInfo ic_emlrtRSI = { 110,/* lineNo */
  "looper",                            /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/datafun/issorted.m"/* pathName */
};

static emlrtRSInfo jc_emlrtRSI = { 93, /* lineNo */
  "looper",                            /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/datafun/issorted.m"/* pathName */
};

/* Function Definitions */
boolean_T issorted(const emlrtStack *sp, const emxArray_int32_T *x)
{
  boolean_T y;
  int32_T dim;
  int32_T n;
  int32_T k;
  boolean_T exitg1;
  int32_T b_n;
  boolean_T exitg2;
  int32_T subs[2];
  emlrtStack st;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  y = true;
  dim = 2;
  if (x->size[0] != 1) {
    dim = 1;
  }

  if (x->size[0] != 0) {
    if (dim <= 1) {
      n = x->size[0];
    } else {
      n = 1;
    }

    if (n != 1) {
      st.site = &hc_emlrtRSI;
      if (dim == 2) {
        n = -1;
      } else {
        n = 0;
      }

      b_st.site = &jc_emlrtRSI;
      k = 0;
      exitg1 = false;
      while ((!exitg1) && (k <= n)) {
        b_st.site = &ic_emlrtRSI;
        if (dim == 1) {
          b_n = x->size[0] - 1;
        } else {
          b_n = x->size[0];
        }

        c_st.site = &jc_emlrtRSI;
        if ((1 <= b_n) && (b_n > 2147483646)) {
          d_st.site = &u_emlrtRSI;
          check_forloop_overflow_error(&d_st);
        }

        k = 0;
        exitg2 = false;
        while ((!exitg2) && (k <= b_n - 1)) {
          subs[0] = k + 1;
          subs[1] = 1;
          subs[dim - 1]++;
          y = ((x->data[k] <= x->data[subs[0] - 1]) && y);
          if (!y) {
            exitg2 = true;
          } else {
            k++;
          }
        }

        if (!y) {
          exitg1 = true;
        } else {
          k = 1;
        }
      }
    }
  }

  return y;
}

/* End of code generation (issorted.cpp) */
