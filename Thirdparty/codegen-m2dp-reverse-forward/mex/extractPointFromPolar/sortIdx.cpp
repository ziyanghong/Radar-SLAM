/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sortIdx.cpp
 *
 * Code generation for function 'sortIdx'
 *
 */

/* Include files */
#include "sortIdx.h"
#include "eml_int_forloop_overflow_check.h"
#include "extractPointFromPolar.h"
#include "extractPointFromPolar_data.h"
#include "extractPointFromPolar_emxutil.h"
#include "rt_nonfinite.h"

/* Variable Definitions */
static emlrtRSInfo gd_emlrtRSI = { 105,/* lineNo */
  "sortIdx",                           /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/sortIdx.m"/* pathName */
};

static emlrtRSInfo hd_emlrtRSI = { 308,/* lineNo */
  "block_merge_sort",                  /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/sortIdx.m"/* pathName */
};

static emlrtRSInfo id_emlrtRSI = { 316,/* lineNo */
  "block_merge_sort",                  /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/sortIdx.m"/* pathName */
};

static emlrtRSInfo jd_emlrtRSI = { 317,/* lineNo */
  "block_merge_sort",                  /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/sortIdx.m"/* pathName */
};

static emlrtRSInfo kd_emlrtRSI = { 325,/* lineNo */
  "block_merge_sort",                  /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/sortIdx.m"/* pathName */
};

static emlrtRSInfo ld_emlrtRSI = { 333,/* lineNo */
  "block_merge_sort",                  /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/sortIdx.m"/* pathName */
};

static emlrtRSInfo md_emlrtRSI = { 443,/* lineNo */
  "initialize_vector_sort",            /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/sortIdx.m"/* pathName */
};

static emlrtRSInfo nd_emlrtRSI = { 468,/* lineNo */
  "initialize_vector_sort",            /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/sortIdx.m"/* pathName */
};

static emlrtRSInfo od_emlrtRSI = { 473,/* lineNo */
  "initialize_vector_sort",            /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/sortIdx.m"/* pathName */
};

static emlrtRSInfo sd_emlrtRSI = { 499,/* lineNo */
  "merge_block",                       /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/sortIdx.m"/* pathName */
};

static emlrtRSInfo ud_emlrtRSI = { 507,/* lineNo */
  "merge_block",                       /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/sortIdx.m"/* pathName */
};

static emlrtRSInfo vd_emlrtRSI = { 514,/* lineNo */
  "merge_block",                       /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/sortIdx.m"/* pathName */
};

static emlrtRSInfo wd_emlrtRSI = { 561,/* lineNo */
  "merge",                             /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/sortIdx.m"/* pathName */
};

static emlrtRSInfo xd_emlrtRSI = { 530,/* lineNo */
  "merge",                             /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/sortIdx.m"/* pathName */
};

static emlrtRTEInfo ff_emlrtRTEI = { 61,/* lineNo */
  5,                                   /* colNo */
  "sortIdx",                           /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/sortIdx.m"/* pName */
};

static emlrtRTEInfo gf_emlrtRTEI = { 386,/* lineNo */
  1,                                   /* colNo */
  "sortIdx",                           /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/sortIdx.m"/* pName */
};

static emlrtRTEInfo hf_emlrtRTEI = { 308,/* lineNo */
  1,                                   /* colNo */
  "sortIdx",                           /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/sortIdx.m"/* pName */
};

static emlrtRTEInfo if_emlrtRTEI = { 308,/* lineNo */
  14,                                  /* colNo */
  "sortIdx",                           /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/sortIdx.m"/* pName */
};

static emlrtRTEInfo jf_emlrtRTEI = { 308,/* lineNo */
  20,                                  /* colNo */
  "sortIdx",                           /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/sortIdx.m"/* pName */
};

/* Function Declarations */
static void merge(const emlrtStack *sp, emxArray_int32_T *idx, emxArray_int32_T *
                  x, int32_T offset, int32_T np, int32_T nq, emxArray_int32_T
                  *iwork, emxArray_int32_T *xwork);
static void merge_block(const emlrtStack *sp, emxArray_int32_T *idx,
  emxArray_int32_T *x, int32_T offset, int32_T n, int32_T preSortLevel,
  emxArray_int32_T *iwork, emxArray_int32_T *xwork);
static void merge_pow2_block(emxArray_int32_T *idx, emxArray_int32_T *x, int32_T
  offset);

/* Function Definitions */
static void merge(const emlrtStack *sp, emxArray_int32_T *idx, emxArray_int32_T *
                  x, int32_T offset, int32_T np, int32_T nq, emxArray_int32_T
                  *iwork, emxArray_int32_T *xwork)
{
  int32_T n_tmp;
  int32_T iout;
  int32_T p;
  int32_T i;
  int32_T q;
  int32_T exitg1;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  if (nq != 0) {
    n_tmp = np + nq;
    st.site = &xd_emlrtRSI;
    if ((1 <= n_tmp) && (n_tmp > 2147483646)) {
      b_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    for (iout = 0; iout < n_tmp; iout++) {
      i = offset + iout;
      iwork->data[iout] = idx->data[i];
      xwork->data[iout] = x->data[i];
    }

    p = 0;
    q = np;
    iout = offset - 1;
    do {
      exitg1 = 0;
      iout++;
      if (xwork->data[p] <= xwork->data[q]) {
        idx->data[iout] = iwork->data[p];
        x->data[iout] = xwork->data[p];
        if (p + 1 < np) {
          p++;
        } else {
          exitg1 = 1;
        }
      } else {
        idx->data[iout] = iwork->data[q];
        x->data[iout] = xwork->data[q];
        if (q + 1 < n_tmp) {
          q++;
        } else {
          q = iout - p;
          st.site = &wd_emlrtRSI;
          if ((p + 1 <= np) && (np > 2147483646)) {
            b_st.site = &u_emlrtRSI;
            check_forloop_overflow_error(&b_st);
          }

          for (iout = p + 1; iout <= np; iout++) {
            i = q + iout;
            idx->data[i] = iwork->data[iout - 1];
            x->data[i] = xwork->data[iout - 1];
          }

          exitg1 = 1;
        }
      }
    } while (exitg1 == 0);
  }
}

static void merge_block(const emlrtStack *sp, emxArray_int32_T *idx,
  emxArray_int32_T *x, int32_T offset, int32_T n, int32_T preSortLevel,
  emxArray_int32_T *iwork, emxArray_int32_T *xwork)
{
  int32_T nPairs;
  int32_T bLen;
  int32_T tailOffset;
  int32_T nTail;
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  nPairs = n >> preSortLevel;
  bLen = 1 << preSortLevel;
  while (nPairs > 1) {
    if ((nPairs & 1) != 0) {
      nPairs--;
      tailOffset = bLen * nPairs;
      nTail = n - tailOffset;
      if (nTail > bLen) {
        st.site = &sd_emlrtRSI;
        merge(&st, idx, x, offset + tailOffset, bLen, nTail - bLen, iwork, xwork);
      }
    }

    tailOffset = bLen << 1;
    nPairs >>= 1;
    for (nTail = 0; nTail < nPairs; nTail++) {
      st.site = &ud_emlrtRSI;
      merge(&st, idx, x, offset + nTail * tailOffset, bLen, bLen, iwork, xwork);
    }

    bLen = tailOffset;
  }

  if (n > bLen) {
    st.site = &vd_emlrtRSI;
    merge(&st, idx, x, offset, bLen, n - bLen, iwork, xwork);
  }
}

static void merge_pow2_block(emxArray_int32_T *idx, emxArray_int32_T *x, int32_T
  offset)
{
  int32_T b;
  int32_T bLen;
  int32_T bLen2;
  int32_T nPairs;
  int32_T k;
  int32_T blockOffset;
  int32_T j;
  int32_T p;
  int32_T iout;
  int32_T q;
  int32_T iwork[256];
  int32_T xwork[256];
  int32_T exitg1;
  for (b = 0; b < 6; b++) {
    bLen = 1 << (b + 2);
    bLen2 = bLen << 1;
    nPairs = 256 >> (b + 3);
    for (k = 0; k < nPairs; k++) {
      blockOffset = offset + k * bLen2;
      for (j = 0; j < bLen2; j++) {
        iout = blockOffset + j;
        iwork[j] = idx->data[iout];
        xwork[j] = x->data[iout];
      }

      p = 0;
      q = bLen;
      iout = blockOffset - 1;
      do {
        exitg1 = 0;
        iout++;
        if (xwork[p] <= xwork[q]) {
          idx->data[iout] = iwork[p];
          x->data[iout] = xwork[p];
          if (p + 1 < bLen) {
            p++;
          } else {
            exitg1 = 1;
          }
        } else {
          idx->data[iout] = iwork[q];
          x->data[iout] = xwork[q];
          if (q + 1 < bLen2) {
            q++;
          } else {
            iout -= p;
            for (j = p + 1; j <= bLen; j++) {
              q = iout + j;
              idx->data[q] = iwork[j - 1];
              x->data[q] = xwork[j - 1];
            }

            exitg1 = 1;
          }
        }
      } while (exitg1 == 0);
    }
  }
}

void sortIdx(const emlrtStack *sp, emxArray_int32_T *x, emxArray_int32_T *idx)
{
  int32_T i3;
  int32_T x4_tmp;
  emxArray_int32_T *iwork;
  int32_T n;
  int32_T b_n;
  int32_T x4[4];
  int32_T idx4[4];
  emxArray_int32_T *xwork;
  int32_T nQuartets;
  int32_T j;
  int32_T nDone;
  int32_T i;
  int32_T i4;
  int8_T perm[4];
  int32_T i1;
  int32_T i2;
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
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  i3 = x->size[0];
  x4_tmp = idx->size[0];
  idx->size[0] = i3;
  emxEnsureCapacity_int32_T(sp, idx, x4_tmp, &ff_emlrtRTEI);
  for (x4_tmp = 0; x4_tmp < i3; x4_tmp++) {
    idx->data[x4_tmp] = 0;
  }

  if (x->size[0] != 0) {
    emxInit_int32_T(sp, &iwork, 1, &if_emlrtRTEI, true);
    st.site = &gd_emlrtRSI;
    n = x->size[0];
    b_st.site = &hd_emlrtRSI;
    b_n = x->size[0] - 1;
    x4[0] = 0;
    idx4[0] = 0;
    x4[1] = 0;
    idx4[1] = 0;
    x4[2] = 0;
    idx4[2] = 0;
    x4[3] = 0;
    idx4[3] = 0;
    x4_tmp = iwork->size[0];
    iwork->size[0] = i3;
    emxEnsureCapacity_int32_T(&b_st, iwork, x4_tmp, &gf_emlrtRTEI);
    for (x4_tmp = 0; x4_tmp < i3; x4_tmp++) {
      iwork->data[x4_tmp] = 0;
    }

    emxInit_int32_T(&b_st, &xwork, 1, &jf_emlrtRTEI, true);
    x4_tmp = xwork->size[0];
    xwork->size[0] = x->size[0];
    emxEnsureCapacity_int32_T(&b_st, xwork, x4_tmp, &hf_emlrtRTEI);
    i3 = xwork->size[0];
    for (x4_tmp = 0; x4_tmp < i3; x4_tmp++) {
      xwork->data[x4_tmp] = 0;
    }

    nQuartets = x->size[0] >> 2;
    c_st.site = &md_emlrtRSI;
    for (j = 0; j < nQuartets; j++) {
      i = j << 2;
      idx4[0] = i + 1;
      idx4[1] = i + 2;
      idx4[2] = i + 3;
      idx4[3] = i + 4;
      x4[0] = x->data[i];
      i3 = x->data[i + 1];
      x4[1] = i3;
      i4 = x->data[i + 2];
      x4[2] = i4;
      x4_tmp = x->data[i + 3];
      x4[3] = x4_tmp;
      if (x->data[i] <= i3) {
        i1 = 1;
        i2 = 2;
      } else {
        i1 = 2;
        i2 = 1;
      }

      if (i4 <= x4_tmp) {
        i3 = 3;
        i4 = 4;
      } else {
        i3 = 4;
        i4 = 3;
      }

      x4_tmp = x4[i1 - 1];
      nDone = x4[i3 - 1];
      if (x4_tmp <= nDone) {
        x4_tmp = x4[i2 - 1];
        if (x4_tmp <= nDone) {
          perm[0] = static_cast<int8_T>(i1);
          perm[1] = static_cast<int8_T>(i2);
          perm[2] = static_cast<int8_T>(i3);
          perm[3] = static_cast<int8_T>(i4);
        } else if (x4_tmp <= x4[i4 - 1]) {
          perm[0] = static_cast<int8_T>(i1);
          perm[1] = static_cast<int8_T>(i3);
          perm[2] = static_cast<int8_T>(i2);
          perm[3] = static_cast<int8_T>(i4);
        } else {
          perm[0] = static_cast<int8_T>(i1);
          perm[1] = static_cast<int8_T>(i3);
          perm[2] = static_cast<int8_T>(i4);
          perm[3] = static_cast<int8_T>(i2);
        }
      } else {
        nDone = x4[i4 - 1];
        if (x4_tmp <= nDone) {
          if (x4[i2 - 1] <= nDone) {
            perm[0] = static_cast<int8_T>(i3);
            perm[1] = static_cast<int8_T>(i1);
            perm[2] = static_cast<int8_T>(i2);
            perm[3] = static_cast<int8_T>(i4);
          } else {
            perm[0] = static_cast<int8_T>(i3);
            perm[1] = static_cast<int8_T>(i1);
            perm[2] = static_cast<int8_T>(i4);
            perm[3] = static_cast<int8_T>(i2);
          }
        } else {
          perm[0] = static_cast<int8_T>(i3);
          perm[1] = static_cast<int8_T>(i4);
          perm[2] = static_cast<int8_T>(i1);
          perm[3] = static_cast<int8_T>(i2);
        }
      }

      i1 = perm[0] - 1;
      idx->data[i] = idx4[i1];
      i2 = perm[1] - 1;
      idx->data[i + 1] = idx4[i2];
      i3 = perm[2] - 1;
      idx->data[i + 2] = idx4[i3];
      i4 = perm[3] - 1;
      idx->data[i + 3] = idx4[i4];
      x->data[i] = x4[i1];
      x->data[i + 1] = x4[i2];
      x->data[i + 2] = x4[i3];
      x->data[i + 3] = x4[i4];
    }

    nDone = nQuartets << 2;
    i4 = (b_n - nDone) + 1;
    if (i4 > 0) {
      c_st.site = &nd_emlrtRSI;
      if (i4 > 2147483646) {
        d_st.site = &u_emlrtRSI;
        check_forloop_overflow_error(&d_st);
      }

      for (x4_tmp = 0; x4_tmp < i4; x4_tmp++) {
        i3 = nDone + x4_tmp;
        idx4[x4_tmp] = i3 + 1;
        x4[x4_tmp] = x->data[i3];
      }

      perm[1] = 0;
      perm[2] = 0;
      perm[3] = 0;
      if (i4 == 1) {
        perm[0] = 1;
      } else if (i4 == 2) {
        if (x4[0] <= x4[1]) {
          perm[0] = 1;
          perm[1] = 2;
        } else {
          perm[0] = 2;
          perm[1] = 1;
        }
      } else if (x4[0] <= x4[1]) {
        if (x4[1] <= x4[2]) {
          perm[0] = 1;
          perm[1] = 2;
          perm[2] = 3;
        } else if (x4[0] <= x4[2]) {
          perm[0] = 1;
          perm[1] = 3;
          perm[2] = 2;
        } else {
          perm[0] = 3;
          perm[1] = 1;
          perm[2] = 2;
        }
      } else if (x4[0] <= x4[2]) {
        perm[0] = 2;
        perm[1] = 1;
        perm[2] = 3;
      } else if (x4[1] <= x4[2]) {
        perm[0] = 2;
        perm[1] = 3;
        perm[2] = 1;
      } else {
        perm[0] = 3;
        perm[1] = 2;
        perm[2] = 1;
      }

      c_st.site = &od_emlrtRSI;
      for (x4_tmp = 0; x4_tmp < i4; x4_tmp++) {
        i1 = perm[x4_tmp] - 1;
        i2 = nDone + x4_tmp;
        idx->data[i2] = idx4[i1];
        x->data[i2] = x4[i1];
      }
    }

    i3 = 2;
    if (n > 1) {
      if (n >= 256) {
        i3 = n >> 8;
        b_st.site = &id_emlrtRSI;
        for (i4 = 0; i4 < i3; i4++) {
          b_st.site = &jd_emlrtRSI;
          merge_pow2_block(idx, x, i4 << 8);
        }

        i3 <<= 8;
        i4 = n - i3;
        if (i4 > 0) {
          b_st.site = &kd_emlrtRSI;
          merge_block(&b_st, idx, x, i3, i4, 2, iwork, xwork);
        }

        i3 = 8;
      }

      b_st.site = &ld_emlrtRSI;
      merge_block(&b_st, idx, x, 0, n, i3, iwork, xwork);
    }

    emxFree_int32_T(&xwork);
    emxFree_int32_T(&iwork);
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (sortIdx.cpp) */
