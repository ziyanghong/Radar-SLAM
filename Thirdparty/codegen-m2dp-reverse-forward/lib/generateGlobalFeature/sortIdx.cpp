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
#include "generateGlobalFeature.h"
#include "generateGlobalFeature_emxutil.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static void merge(emxArray_int32_T *idx, emxArray_int32_T *x, int offset, int np,
                  int nq, emxArray_int32_T *iwork, emxArray_int32_T *xwork);
static void merge_block(emxArray_int32_T *idx, emxArray_int32_T *x, int offset,
  int n, int preSortLevel, emxArray_int32_T *iwork, emxArray_int32_T *xwork);

/* Function Definitions */
static void merge(emxArray_int32_T *idx, emxArray_int32_T *x, int offset, int np,
                  int nq, emxArray_int32_T *iwork, emxArray_int32_T *xwork)
{
  int n_tmp;
  int iout;
  int p;
  int i;
  int q;
  int exitg1;
  if (nq != 0) {
    n_tmp = np + nq;
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

static void merge_block(emxArray_int32_T *idx, emxArray_int32_T *x, int offset,
  int n, int preSortLevel, emxArray_int32_T *iwork, emxArray_int32_T *xwork)
{
  int nPairs;
  int bLen;
  int tailOffset;
  int nTail;
  nPairs = n >> preSortLevel;
  bLen = 1 << preSortLevel;
  while (nPairs > 1) {
    if ((nPairs & 1) != 0) {
      nPairs--;
      tailOffset = bLen * nPairs;
      nTail = n - tailOffset;
      if (nTail > bLen) {
        merge(idx, x, offset + tailOffset, bLen, nTail - bLen, iwork, xwork);
      }
    }

    tailOffset = bLen << 1;
    nPairs >>= 1;
    for (nTail = 0; nTail < nPairs; nTail++) {
      merge(idx, x, offset + nTail * tailOffset, bLen, bLen, iwork, xwork);
    }

    bLen = tailOffset;
  }

  if (n > bLen) {
    merge(idx, x, offset, bLen, n - bLen, iwork, xwork);
  }
}

void sortIdx(emxArray_int32_T *x, emxArray_int32_T *idx)
{
  int i3;
  int p;
  emxArray_int32_T *iwork;
  int n;
  int b_n;
  int x4[4];
  int idx4[4];
  emxArray_int32_T *xwork;
  int nQuartets;
  int j;
  int nDone;
  int i;
  int i4;
  int k;
  signed char perm[4];
  int i1;
  int i2;
  int idx_tmp;
  int bLen2;
  int nPairs;
  int b_iwork[256];
  int b_xwork[256];
  int exitg1;
  i3 = x->size[0];
  p = idx->size[0];
  idx->size[0] = i3;
  emxEnsureCapacity_int32_T(idx, p);
  for (p = 0; p < i3; p++) {
    idx->data[p] = 0;
  }

  if (x->size[0] != 0) {
    emxInit_int32_T(&iwork, 1);
    n = x->size[0];
    b_n = x->size[0] - 1;
    x4[0] = 0;
    idx4[0] = 0;
    x4[1] = 0;
    idx4[1] = 0;
    x4[2] = 0;
    idx4[2] = 0;
    x4[3] = 0;
    idx4[3] = 0;
    p = iwork->size[0];
    iwork->size[0] = i3;
    emxEnsureCapacity_int32_T(iwork, p);
    for (p = 0; p < i3; p++) {
      iwork->data[p] = 0;
    }

    emxInit_int32_T(&xwork, 1);
    p = xwork->size[0];
    xwork->size[0] = x->size[0];
    emxEnsureCapacity_int32_T(xwork, p);
    i3 = xwork->size[0];
    for (p = 0; p < i3; p++) {
      xwork->data[p] = 0;
    }

    nQuartets = x->size[0] >> 2;
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
      nDone = x->data[i + 3];
      x4[3] = nDone;
      if (x->data[i] <= i3) {
        i1 = 1;
        i2 = 2;
      } else {
        i1 = 2;
        i2 = 1;
      }

      if (i4 <= nDone) {
        i3 = 3;
        i4 = 4;
      } else {
        i3 = 4;
        i4 = 3;
      }

      p = x4[i1 - 1];
      nDone = x4[i3 - 1];
      if (p <= nDone) {
        p = x4[i2 - 1];
        if (p <= nDone) {
          perm[0] = static_cast<signed char>(i1);
          perm[1] = static_cast<signed char>(i2);
          perm[2] = static_cast<signed char>(i3);
          perm[3] = static_cast<signed char>(i4);
        } else if (p <= x4[i4 - 1]) {
          perm[0] = static_cast<signed char>(i1);
          perm[1] = static_cast<signed char>(i3);
          perm[2] = static_cast<signed char>(i2);
          perm[3] = static_cast<signed char>(i4);
        } else {
          perm[0] = static_cast<signed char>(i1);
          perm[1] = static_cast<signed char>(i3);
          perm[2] = static_cast<signed char>(i4);
          perm[3] = static_cast<signed char>(i2);
        }
      } else {
        nDone = x4[i4 - 1];
        if (p <= nDone) {
          if (x4[i2 - 1] <= nDone) {
            perm[0] = static_cast<signed char>(i3);
            perm[1] = static_cast<signed char>(i1);
            perm[2] = static_cast<signed char>(i2);
            perm[3] = static_cast<signed char>(i4);
          } else {
            perm[0] = static_cast<signed char>(i3);
            perm[1] = static_cast<signed char>(i1);
            perm[2] = static_cast<signed char>(i4);
            perm[3] = static_cast<signed char>(i2);
          }
        } else {
          perm[0] = static_cast<signed char>(i3);
          perm[1] = static_cast<signed char>(i4);
          perm[2] = static_cast<signed char>(i1);
          perm[3] = static_cast<signed char>(i2);
        }
      }

      idx_tmp = perm[0] - 1;
      idx->data[i] = idx4[idx_tmp];
      p = perm[1] - 1;
      idx->data[i + 1] = idx4[p];
      i3 = perm[2] - 1;
      idx->data[i + 2] = idx4[i3];
      i4 = perm[3] - 1;
      idx->data[i + 3] = idx4[i4];
      x->data[i] = x4[idx_tmp];
      x->data[i + 1] = x4[p];
      x->data[i + 2] = x4[i3];
      x->data[i + 3] = x4[i4];
    }

    nDone = nQuartets << 2;
    i4 = b_n - nDone;
    if (i4 + 1 > 0) {
      for (k = 0; k <= i4; k++) {
        i3 = nDone + k;
        idx4[k] = i3 + 1;
        x4[k] = x->data[i3];
      }

      perm[1] = 0;
      perm[2] = 0;
      perm[3] = 0;
      if (i4 + 1 == 1) {
        perm[0] = 1;
      } else if (i4 + 1 == 2) {
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

      for (k = 0; k <= i4; k++) {
        idx_tmp = perm[k] - 1;
        p = nDone + k;
        idx->data[p] = idx4[idx_tmp];
        x->data[p] = x4[idx_tmp];
      }
    }

    i3 = 2;
    if (n > 1) {
      if (n >= 256) {
        i1 = n >> 8;
        for (i2 = 0; i2 < i1; i2++) {
          nQuartets = (i2 << 8) - 1;
          for (i = 0; i < 6; i++) {
            b_n = 1 << (i + 2);
            bLen2 = b_n << 1;
            nPairs = 256 >> (i + 3);
            for (k = 0; k < nPairs; k++) {
              i4 = (nQuartets + k * bLen2) + 1;
              for (j = 0; j < bLen2; j++) {
                nDone = i4 + j;
                b_iwork[j] = idx->data[nDone];
                b_xwork[j] = x->data[nDone];
              }

              p = 0;
              i3 = b_n;
              nDone = i4 - 1;
              do {
                exitg1 = 0;
                nDone++;
                if (b_xwork[p] <= b_xwork[i3]) {
                  idx->data[nDone] = b_iwork[p];
                  x->data[nDone] = b_xwork[p];
                  if (p + 1 < b_n) {
                    p++;
                  } else {
                    exitg1 = 1;
                  }
                } else {
                  idx->data[nDone] = b_iwork[i3];
                  x->data[nDone] = b_xwork[i3];
                  if (i3 + 1 < bLen2) {
                    i3++;
                  } else {
                    nDone -= p;
                    for (j = p + 1; j <= b_n; j++) {
                      idx_tmp = nDone + j;
                      idx->data[idx_tmp] = b_iwork[j - 1];
                      x->data[idx_tmp] = b_xwork[j - 1];
                    }

                    exitg1 = 1;
                  }
                }
              } while (exitg1 == 0);
            }
          }
        }

        i3 = i1 << 8;
        i4 = n - i3;
        if (i4 > 0) {
          merge_block(idx, x, i3, i4, 2, iwork, xwork);
        }

        i3 = 8;
      }

      merge_block(idx, x, 0, n, i3, iwork, xwork);
    }

    emxFree_int32_T(&xwork);
    emxFree_int32_T(&iwork);
  }
}

/* End of code generation (sortIdx.cpp) */
