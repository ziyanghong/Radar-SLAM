/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * findpeaks.cpp
 *
 * Code generation for function 'findpeaks'
 *
 */

/* Include files */
#include "findpeaks.h"
#include "eml_setop.h"
#include "generateGlobalFeature.h"
#include "generateGlobalFeature_emxutil.h"
#include "rt_nonfinite.h"
#include "sort.h"

/* Function Declarations */
static void findExtents(const emxArray_real_T *y, const emxArray_real_T *x,
  emxArray_int32_T *iPk, const emxArray_int32_T *iFin, const emxArray_int32_T
  *iInf, const emxArray_int32_T *iInflect, emxArray_real_T *bPk, emxArray_real_T
  *bxPk, emxArray_real_T *byPk, emxArray_real_T *wxPk);
static void getLeftBase(const emxArray_real_T *yTemp, const emxArray_int32_T
  *iPeak, const emxArray_int32_T *iFinite, const emxArray_int32_T *iInflect,
  emxArray_int32_T *iBase, emxArray_int32_T *iSaddle);

/* Function Definitions */
static void findExtents(const emxArray_real_T *y, const emxArray_real_T *x,
  emxArray_int32_T *iPk, const emxArray_int32_T *iFin, const emxArray_int32_T
  *iInf, const emxArray_int32_T *iInflect, emxArray_real_T *bPk, emxArray_real_T
  *bxPk, emxArray_real_T *byPk, emxArray_real_T *wxPk)
{
  emxArray_real_T *yFinite;
  int xtmp;
  int iLeft;
  emxArray_int32_T *iLeftSaddle;
  emxArray_int32_T *ii;
  emxArray_int32_T *b_x;
  int m;
  int md2;
  int i;
  emxArray_int32_T *c_x;
  emxArray_int32_T *d_x;
  emxArray_int32_T *iRightBase;
  emxArray_int32_T *iRightSaddle;
  emxArray_real_T *b_bPk;
  double xc_tmp;
  emxArray_boolean_T *e_x;
  double refHeight_tmp;
  boolean_T exitg1;
  emxArray_real_T *idx;
  emxArray_real_T *b_wxPk;
  double refHeight;
  double b_xc_tmp;
  emxArray_int32_T *iInfL;
  emxArray_int32_T *iInfR;
  emxInit_real_T(&yFinite, 1);
  xtmp = yFinite->size[0];
  yFinite->size[0] = y->size[0];
  emxEnsureCapacity_real_T(yFinite, xtmp);
  iLeft = y->size[0];
  for (xtmp = 0; xtmp < iLeft; xtmp++) {
    yFinite->data[xtmp] = y->data[xtmp];
  }

  iLeft = iInf->size[0];
  for (xtmp = 0; xtmp < iLeft; xtmp++) {
    yFinite->data[iInf->data[xtmp] - 1] = rtNaN;
  }

  emxInit_int32_T(&iLeftSaddle, 1);
  emxInit_int32_T(&ii, 1);
  emxInit_int32_T(&b_x, 1);
  getLeftBase(yFinite, iPk, iFin, iInflect, ii, iLeftSaddle);
  xtmp = b_x->size[0];
  b_x->size[0] = iPk->size[0];
  emxEnsureCapacity_int32_T(b_x, xtmp);
  iLeft = iPk->size[0];
  for (xtmp = 0; xtmp < iLeft; xtmp++) {
    b_x->data[xtmp] = iPk->data[xtmp];
  }

  m = iPk->size[0] - 1;
  md2 = iPk->size[0] >> 1;
  for (i = 0; i < md2; i++) {
    xtmp = b_x->data[i];
    iLeft = m - i;
    b_x->data[i] = b_x->data[iLeft];
    b_x->data[iLeft] = xtmp;
  }

  emxInit_int32_T(&c_x, 1);
  xtmp = c_x->size[0];
  c_x->size[0] = iFin->size[0];
  emxEnsureCapacity_int32_T(c_x, xtmp);
  iLeft = iFin->size[0];
  for (xtmp = 0; xtmp < iLeft; xtmp++) {
    c_x->data[xtmp] = iFin->data[xtmp];
  }

  m = iFin->size[0] - 1;
  md2 = iFin->size[0] >> 1;
  for (i = 0; i < md2; i++) {
    xtmp = c_x->data[i];
    iLeft = m - i;
    c_x->data[i] = c_x->data[iLeft];
    c_x->data[iLeft] = xtmp;
  }

  emxInit_int32_T(&d_x, 1);
  xtmp = d_x->size[0];
  d_x->size[0] = iInflect->size[0];
  emxEnsureCapacity_int32_T(d_x, xtmp);
  iLeft = iInflect->size[0];
  for (xtmp = 0; xtmp < iLeft; xtmp++) {
    d_x->data[xtmp] = iInflect->data[xtmp];
  }

  m = iInflect->size[0] - 1;
  md2 = iInflect->size[0] >> 1;
  for (i = 0; i < md2; i++) {
    xtmp = d_x->data[i];
    iLeft = m - i;
    d_x->data[i] = d_x->data[iLeft];
    d_x->data[iLeft] = xtmp;
  }

  emxInit_int32_T(&iRightBase, 1);
  emxInit_int32_T(&iRightSaddle, 1);
  getLeftBase(yFinite, b_x, c_x, d_x, iRightBase, iRightSaddle);
  m = iRightBase->size[0] - 1;
  md2 = iRightBase->size[0] >> 1;
  for (i = 0; i < md2; i++) {
    xtmp = iRightBase->data[i];
    iLeft = m - i;
    iRightBase->data[i] = iRightBase->data[iLeft];
    iRightBase->data[iLeft] = xtmp;
  }

  m = iRightSaddle->size[0] - 1;
  md2 = iRightSaddle->size[0] >> 1;
  for (i = 0; i < md2; i++) {
    xtmp = iRightSaddle->data[i];
    iLeft = m - i;
    iRightSaddle->data[i] = iRightSaddle->data[iLeft];
    iRightSaddle->data[iLeft] = xtmp;
  }

  emxInit_real_T(&b_bPk, 1);
  xtmp = b_bPk->size[0];
  if (ii->size[0] <= iRightBase->size[0]) {
    b_bPk->size[0] = ii->size[0];
  } else {
    b_bPk->size[0] = iRightBase->size[0];
  }

  emxEnsureCapacity_real_T(b_bPk, xtmp);
  if (ii->size[0] <= iRightBase->size[0]) {
    md2 = ii->size[0];
  } else {
    md2 = iRightBase->size[0];
  }

  for (iLeft = 0; iLeft < md2; iLeft++) {
    xc_tmp = yFinite->data[iRightBase->data[iLeft] - 1];
    refHeight_tmp = yFinite->data[ii->data[iLeft] - 1];
    if ((refHeight_tmp > xc_tmp) || rtIsNaN(xc_tmp)) {
      b_bPk->data[iLeft] = refHeight_tmp;
    } else {
      b_bPk->data[iLeft] = xc_tmp;
    }
  }

  emxFree_int32_T(&iRightBase);
  emxInit_boolean_T(&e_x, 1);
  xtmp = e_x->size[0];
  e_x->size[0] = iPk->size[0];
  emxEnsureCapacity_boolean_T(e_x, xtmp);
  iLeft = iPk->size[0];
  for (xtmp = 0; xtmp < iLeft; xtmp++) {
    e_x->data[xtmp] = (yFinite->data[iPk->data[xtmp] - 1] - b_bPk->data[xtmp] >=
                       8.0);
  }

  md2 = e_x->size[0];
  m = 0;
  xtmp = ii->size[0];
  ii->size[0] = e_x->size[0];
  emxEnsureCapacity_int32_T(ii, xtmp);
  iLeft = 0;
  exitg1 = false;
  while ((!exitg1) && (iLeft <= md2 - 1)) {
    if (e_x->data[iLeft]) {
      m++;
      ii->data[m - 1] = iLeft + 1;
      if (m >= md2) {
        exitg1 = true;
      } else {
        iLeft++;
      }
    } else {
      iLeft++;
    }
  }

  if (e_x->size[0] == 1) {
    if (m == 0) {
      ii->size[0] = 0;
    }
  } else {
    xtmp = ii->size[0];
    if (1 > m) {
      ii->size[0] = 0;
    } else {
      ii->size[0] = m;
    }

    emxEnsureCapacity_int32_T(ii, xtmp);
  }

  emxFree_boolean_T(&e_x);
  emxInit_real_T(&idx, 1);
  xtmp = idx->size[0];
  idx->size[0] = ii->size[0];
  emxEnsureCapacity_real_T(idx, xtmp);
  iLeft = ii->size[0];
  for (xtmp = 0; xtmp < iLeft; xtmp++) {
    idx->data[xtmp] = ii->data[xtmp];
  }

  xtmp = ii->size[0];
  ii->size[0] = idx->size[0];
  emxEnsureCapacity_int32_T(ii, xtmp);
  iLeft = idx->size[0];
  for (xtmp = 0; xtmp < iLeft; xtmp++) {
    ii->data[xtmp] = iPk->data[static_cast<int>(idx->data[xtmp]) - 1];
  }

  xtmp = iPk->size[0];
  iPk->size[0] = ii->size[0];
  emxEnsureCapacity_int32_T(iPk, xtmp);
  iLeft = ii->size[0];
  for (xtmp = 0; xtmp < iLeft; xtmp++) {
    iPk->data[xtmp] = ii->data[xtmp];
  }

  xtmp = bPk->size[0];
  bPk->size[0] = idx->size[0];
  emxEnsureCapacity_real_T(bPk, xtmp);
  iLeft = idx->size[0];
  for (xtmp = 0; xtmp < iLeft; xtmp++) {
    bPk->data[xtmp] = b_bPk->data[static_cast<int>(idx->data[xtmp]) - 1];
  }

  xtmp = b_bPk->size[0];
  b_bPk->size[0] = bPk->size[0];
  emxEnsureCapacity_real_T(b_bPk, xtmp);
  iLeft = bPk->size[0];
  for (xtmp = 0; xtmp < iLeft; xtmp++) {
    b_bPk->data[xtmp] = bPk->data[xtmp];
  }

  xtmp = ii->size[0];
  ii->size[0] = idx->size[0];
  emxEnsureCapacity_int32_T(ii, xtmp);
  iLeft = idx->size[0];
  for (xtmp = 0; xtmp < iLeft; xtmp++) {
    ii->data[xtmp] = iLeftSaddle->data[static_cast<int>(idx->data[xtmp]) - 1];
  }

  xtmp = iLeftSaddle->size[0];
  iLeftSaddle->size[0] = ii->size[0];
  emxEnsureCapacity_int32_T(iLeftSaddle, xtmp);
  iLeft = ii->size[0];
  for (xtmp = 0; xtmp < iLeft; xtmp++) {
    iLeftSaddle->data[xtmp] = ii->data[xtmp];
  }

  xtmp = ii->size[0];
  ii->size[0] = idx->size[0];
  emxEnsureCapacity_int32_T(ii, xtmp);
  iLeft = idx->size[0];
  for (xtmp = 0; xtmp < iLeft; xtmp++) {
    ii->data[xtmp] = iRightSaddle->data[static_cast<int>(idx->data[xtmp]) - 1];
  }

  xtmp = iRightSaddle->size[0];
  iRightSaddle->size[0] = ii->size[0];
  emxEnsureCapacity_int32_T(iRightSaddle, xtmp);
  iLeft = ii->size[0];
  for (xtmp = 0; xtmp < iLeft; xtmp++) {
    iRightSaddle->data[xtmp] = ii->data[xtmp];
  }

  if (iPk->size[0] == 0) {
    idx->size[0] = 0;
    iLeftSaddle->size[0] = 0;
    iRightSaddle->size[0] = 0;
  } else {
    xtmp = idx->size[0];
    idx->size[0] = b_bPk->size[0];
    emxEnsureCapacity_real_T(idx, xtmp);
    iLeft = b_bPk->size[0];
    for (xtmp = 0; xtmp < iLeft; xtmp++) {
      idx->data[xtmp] = b_bPk->data[xtmp];
    }
  }

  emxInit_real_T(&b_wxPk, 2);
  xtmp = b_wxPk->size[0] * b_wxPk->size[1];
  b_wxPk->size[0] = iPk->size[0];
  b_wxPk->size[1] = 2;
  emxEnsureCapacity_real_T(b_wxPk, xtmp);
  iLeft = iPk->size[0] << 1;
  for (xtmp = 0; xtmp < iLeft; xtmp++) {
    b_wxPk->data[xtmp] = 0.0;
  }

  xtmp = iPk->size[0];
  for (i = 0; i < xtmp; i++) {
    refHeight_tmp = yFinite->data[iPk->data[i] - 1] + idx->data[i];
    refHeight = refHeight_tmp / 2.0;
    iLeft = iPk->data[i];
    while ((iLeft >= iLeftSaddle->data[i]) && (yFinite->data[iLeft - 1] >
            refHeight)) {
      iLeft--;
    }

    if (iLeft < iLeftSaddle->data[i]) {
      b_wxPk->data[i] = x->data[iLeftSaddle->data[i] - 1];
    } else {
      b_xc_tmp = yFinite->data[iLeft - 1];
      xc_tmp = x->data[iLeft - 1];
      xc_tmp += (x->data[iLeft] - xc_tmp) * (0.5 * refHeight_tmp - b_xc_tmp) /
        (yFinite->data[iLeft] - b_xc_tmp);
      if (rtIsNaN(xc_tmp)) {
        xc_tmp = x->data[iLeft];
      }

      b_wxPk->data[i] = xc_tmp;
    }

    iLeft = iPk->data[i] - 1;
    while ((iLeft + 1 <= iRightSaddle->data[i]) && (yFinite->data[iLeft] >
            refHeight)) {
      iLeft++;
    }

    if (iLeft + 1 > iRightSaddle->data[i]) {
      b_wxPk->data[i + b_wxPk->size[0]] = x->data[iRightSaddle->data[i] - 1];
    } else {
      b_xc_tmp = x->data[iLeft - 1];
      xc_tmp = x->data[iLeft] + (b_xc_tmp - x->data[iLeft]) * (0.5 *
        (yFinite->data[iPk->data[i] - 1] + idx->data[i]) - yFinite->data[iLeft])
        / (yFinite->data[iLeft - 1] - yFinite->data[iLeft]);
      if (rtIsNaN(xc_tmp)) {
        xc_tmp = b_xc_tmp;
      }

      b_wxPk->data[i + b_wxPk->size[0]] = xc_tmp;
    }
  }

  xtmp = ii->size[0];
  ii->size[0] = iPk->size[0];
  emxEnsureCapacity_int32_T(ii, xtmp);
  iLeft = iPk->size[0];
  for (xtmp = 0; xtmp < iLeft; xtmp++) {
    ii->data[xtmp] = iPk->data[xtmp];
  }

  xtmp = d_x->size[0];
  d_x->size[0] = iPk->size[0];
  emxEnsureCapacity_int32_T(d_x, xtmp);
  iLeft = iPk->size[0] - 1;
  for (xtmp = 0; xtmp <= iLeft; xtmp++) {
    d_x->data[xtmp] = iPk->data[xtmp];
  }

  do_vectors(d_x, iInf, iPk, b_x, c_x);
  b_do_vectors(iPk, ii, d_x, b_x, c_x);
  xtmp = idx->size[0];
  idx->size[0] = b_x->size[0];
  emxEnsureCapacity_real_T(idx, xtmp);
  iLeft = b_x->size[0];
  for (xtmp = 0; xtmp < iLeft; xtmp++) {
    idx->data[xtmp] = b_x->data[xtmp];
  }

  b_do_vectors(iPk, iInf, d_x, b_x, c_x);
  xtmp = yFinite->size[0];
  yFinite->size[0] = b_x->size[0];
  emxEnsureCapacity_real_T(yFinite, xtmp);
  iLeft = b_x->size[0];
  emxFree_int32_T(&d_x);
  emxFree_int32_T(&c_x);
  for (xtmp = 0; xtmp < iLeft; xtmp++) {
    yFinite->data[xtmp] = b_x->data[xtmp];
  }

  emxFree_int32_T(&b_x);
  iLeft = iPk->size[0];
  xtmp = bPk->size[0];
  bPk->size[0] = iLeft;
  emxEnsureCapacity_real_T(bPk, xtmp);
  for (xtmp = 0; xtmp < iLeft; xtmp++) {
    bPk->data[xtmp] = 0.0;
  }

  iLeft = b_bPk->size[0];
  for (xtmp = 0; xtmp < iLeft; xtmp++) {
    bPk->data[static_cast<int>(idx->data[xtmp]) - 1] = b_bPk->data[xtmp];
  }

  emxFree_real_T(&b_bPk);
  xtmp = ii->size[0];
  ii->size[0] = yFinite->size[0];
  emxEnsureCapacity_int32_T(ii, xtmp);
  iLeft = yFinite->size[0];
  for (xtmp = 0; xtmp < iLeft; xtmp++) {
    ii->data[xtmp] = static_cast<int>(yFinite->data[xtmp]);
  }

  iLeft = ii->size[0];
  for (xtmp = 0; xtmp < iLeft; xtmp++) {
    bPk->data[ii->data[xtmp] - 1] = 0.0;
  }

  xtmp = ii->size[0];
  ii->size[0] = iInf->size[0];
  emxEnsureCapacity_int32_T(ii, xtmp);
  iLeft = iInf->size[0];
  for (xtmp = 0; xtmp < iLeft; xtmp++) {
    ii->data[xtmp] = iInf->data[xtmp] - 1;
  }

  emxInit_int32_T(&iInfL, 1);
  xtmp = iInfL->size[0];
  iInfL->size[0] = ii->size[0];
  emxEnsureCapacity_int32_T(iInfL, xtmp);
  md2 = ii->size[0];
  for (iLeft = 0; iLeft < md2; iLeft++) {
    if (1 < ii->data[iLeft]) {
      iInfL->data[iLeft] = ii->data[iLeft];
    } else {
      iInfL->data[iLeft] = 1;
    }
  }

  m = x->size[0];
  xtmp = ii->size[0];
  ii->size[0] = iInf->size[0];
  emxEnsureCapacity_int32_T(ii, xtmp);
  iLeft = iInf->size[0];
  for (xtmp = 0; xtmp < iLeft; xtmp++) {
    ii->data[xtmp] = iInf->data[xtmp] + 1;
  }

  emxInit_int32_T(&iInfR, 1);
  xtmp = iInfR->size[0];
  iInfR->size[0] = ii->size[0];
  emxEnsureCapacity_int32_T(iInfR, xtmp);
  md2 = ii->size[0];
  for (iLeft = 0; iLeft < md2; iLeft++) {
    if (ii->data[iLeft] > m) {
      iInfR->data[iLeft] = m;
    } else {
      iInfR->data[iLeft] = ii->data[iLeft];
    }
  }

  emxFree_int32_T(&ii);
  xtmp = bxPk->size[0] * bxPk->size[1];
  bxPk->size[0] = iPk->size[0];
  bxPk->size[1] = 2;
  emxEnsureCapacity_real_T(bxPk, xtmp);
  iLeft = iPk->size[0] << 1;
  for (xtmp = 0; xtmp < iLeft; xtmp++) {
    bxPk->data[xtmp] = 0.0;
  }

  iLeft = iLeftSaddle->size[0];
  for (xtmp = 0; xtmp < iLeft; xtmp++) {
    bxPk->data[static_cast<int>(idx->data[xtmp]) - 1] = x->data
      [iLeftSaddle->data[xtmp] - 1];
  }

  iLeft = iRightSaddle->size[0];
  for (xtmp = 0; xtmp < iLeft; xtmp++) {
    bxPk->data[(static_cast<int>(idx->data[xtmp]) + bxPk->size[0]) - 1] =
      x->data[iRightSaddle->data[xtmp] - 1];
  }

  iLeft = iInf->size[0];
  for (xtmp = 0; xtmp < iLeft; xtmp++) {
    bxPk->data[static_cast<int>(yFinite->data[xtmp]) - 1] = 0.5 * (x->data
      [iInf->data[xtmp] - 1] + x->data[iInfL->data[xtmp] - 1]);
  }

  iLeft = iInf->size[0];
  for (xtmp = 0; xtmp < iLeft; xtmp++) {
    bxPk->data[(static_cast<int>(yFinite->data[xtmp]) + bxPk->size[0]) - 1] =
      0.5 * (x->data[iInf->data[xtmp] - 1] + x->data[iInfR->data[xtmp] - 1]);
  }

  xtmp = byPk->size[0] * byPk->size[1];
  byPk->size[0] = iPk->size[0];
  byPk->size[1] = 2;
  emxEnsureCapacity_real_T(byPk, xtmp);
  iLeft = iPk->size[0] << 1;
  for (xtmp = 0; xtmp < iLeft; xtmp++) {
    byPk->data[xtmp] = 0.0;
  }

  iLeft = iLeftSaddle->size[0];
  for (xtmp = 0; xtmp < iLeft; xtmp++) {
    byPk->data[static_cast<int>(idx->data[xtmp]) - 1] = y->data
      [iLeftSaddle->data[xtmp] - 1];
  }

  emxFree_int32_T(&iLeftSaddle);
  iLeft = iRightSaddle->size[0];
  for (xtmp = 0; xtmp < iLeft; xtmp++) {
    byPk->data[(static_cast<int>(idx->data[xtmp]) + byPk->size[0]) - 1] =
      y->data[iRightSaddle->data[xtmp] - 1];
  }

  emxFree_int32_T(&iRightSaddle);
  iLeft = iInfL->size[0];
  for (xtmp = 0; xtmp < iLeft; xtmp++) {
    byPk->data[static_cast<int>(yFinite->data[xtmp]) - 1] = y->data[iInfL->
      data[xtmp] - 1];
  }

  iLeft = iInfR->size[0];
  for (xtmp = 0; xtmp < iLeft; xtmp++) {
    byPk->data[(static_cast<int>(yFinite->data[xtmp]) + byPk->size[0]) - 1] =
      y->data[iInfR->data[xtmp] - 1];
  }

  xtmp = wxPk->size[0] * wxPk->size[1];
  wxPk->size[0] = iPk->size[0];
  wxPk->size[1] = 2;
  emxEnsureCapacity_real_T(wxPk, xtmp);
  iLeft = iPk->size[0] << 1;
  for (xtmp = 0; xtmp < iLeft; xtmp++) {
    wxPk->data[xtmp] = 0.0;
  }

  iLeft = b_wxPk->size[0];
  for (xtmp = 0; xtmp < iLeft; xtmp++) {
    wxPk->data[static_cast<int>(idx->data[xtmp]) - 1] = b_wxPk->data[xtmp];
  }

  for (xtmp = 0; xtmp < iLeft; xtmp++) {
    wxPk->data[(static_cast<int>(idx->data[xtmp]) + wxPk->size[0]) - 1] =
      b_wxPk->data[xtmp + b_wxPk->size[0]];
  }

  emxFree_real_T(&idx);
  emxFree_real_T(&b_wxPk);
  iLeft = iInf->size[0];
  for (xtmp = 0; xtmp < iLeft; xtmp++) {
    wxPk->data[static_cast<int>(yFinite->data[xtmp]) - 1] = 0.5 * (x->data
      [iInf->data[xtmp] - 1] + x->data[iInfL->data[xtmp] - 1]);
  }

  emxFree_int32_T(&iInfL);
  iLeft = iInf->size[0];
  for (xtmp = 0; xtmp < iLeft; xtmp++) {
    wxPk->data[(static_cast<int>(yFinite->data[xtmp]) + wxPk->size[0]) - 1] =
      0.5 * (x->data[iInf->data[xtmp] - 1] + x->data[iInfR->data[xtmp] - 1]);
  }

  emxFree_int32_T(&iInfR);
  emxFree_real_T(&yFinite);
}

static void getLeftBase(const emxArray_real_T *yTemp, const emxArray_int32_T
  *iPeak, const emxArray_int32_T *iFinite, const emxArray_int32_T *iInflect,
  emxArray_int32_T *iBase, emxArray_int32_T *iSaddle)
{
  int n;
  int i;
  emxArray_real_T *peak;
  emxArray_real_T *valley;
  emxArray_int32_T *iValley;
  int j;
  int k;
  double v;
  int iv;
  double p_tmp;
  int isv;
  n = iBase->size[0];
  iBase->size[0] = iPeak->size[0];
  emxEnsureCapacity_int32_T(iBase, n);
  i = iPeak->size[0];
  for (n = 0; n < i; n++) {
    iBase->data[n] = 0;
  }

  n = iSaddle->size[0];
  iSaddle->size[0] = iPeak->size[0];
  emxEnsureCapacity_int32_T(iSaddle, n);
  i = iPeak->size[0];
  for (n = 0; n < i; n++) {
    iSaddle->data[n] = 0;
  }

  emxInit_real_T(&peak, 1);
  n = peak->size[0];
  peak->size[0] = iFinite->size[0];
  emxEnsureCapacity_real_T(peak, n);
  i = iFinite->size[0];
  for (n = 0; n < i; n++) {
    peak->data[n] = 0.0;
  }

  emxInit_real_T(&valley, 1);
  n = valley->size[0];
  valley->size[0] = iFinite->size[0];
  emxEnsureCapacity_real_T(valley, n);
  i = iFinite->size[0];
  for (n = 0; n < i; n++) {
    valley->data[n] = 0.0;
  }

  emxInit_int32_T(&iValley, 1);
  n = iValley->size[0];
  iValley->size[0] = iFinite->size[0];
  emxEnsureCapacity_int32_T(iValley, n);
  i = iFinite->size[0];
  for (n = 0; n < i; n++) {
    iValley->data[n] = 0;
  }

  n = -1;
  i = 0;
  j = 0;
  k = 0;
  v = rtNaN;
  iv = 1;
  while (k + 1 <= iPeak->size[0]) {
    while (iInflect->data[i] != iFinite->data[j]) {
      v = yTemp->data[iInflect->data[i] - 1];
      iv = iInflect->data[i];
      if (rtIsNaN(v)) {
        n = -1;
      } else {
        while ((n + 1 > 0) && (valley->data[n] > v)) {
          n--;
        }
      }

      i++;
    }

    p_tmp = yTemp->data[iInflect->data[i] - 1];
    while ((n + 1 > 0) && (peak->data[n] < p_tmp)) {
      if (valley->data[n] < v) {
        v = valley->data[n];
        iv = iValley->data[n];
      }

      n--;
    }

    isv = iv;
    while ((n + 1 > 0) && (peak->data[n] <= p_tmp)) {
      if (valley->data[n] < v) {
        v = valley->data[n];
        iv = iValley->data[n];
      }

      n--;
    }

    n++;
    peak->data[n] = p_tmp;
    valley->data[n] = v;
    iValley->data[n] = iv;
    if (iInflect->data[i] == iPeak->data[k]) {
      iBase->data[k] = iv;
      iSaddle->data[k] = isv;
      k++;
    }

    i++;
    j++;
  }

  emxFree_int32_T(&iValley);
  emxFree_real_T(&valley);
  emxFree_real_T(&peak);
}

void findpeaks(const emxArray_real_T *Yin, const emxArray_real_T *varargin_1,
               emxArray_real_T *Ypk, emxArray_real_T *Xpk)
{
  emxArray_int32_T *idx;
  emxArray_int32_T *iInflect;
  emxArray_int32_T *b_sortIdx;
  int i;
  int ny;
  int nPk;
  int nInflect;
  char dir;
  int kfirst;
  double ykfirst;
  boolean_T isinfykfirst;
  int k;
  double yk;
  emxArray_int32_T *iPk;
  boolean_T isinfyk;
  char previousdir;
  emxArray_int32_T *b_iPk;
  emxArray_real_T *bxPk;
  emxArray_real_T *byPk;
  emxArray_real_T *wxPk;
  emxArray_real_T *locs_temp;
  emxArray_real_T b_Yin;
  int c_Yin[1];
  emxArray_real_T b_varargin_1;
  int c_varargin_1[1];
  int n;
  emxArray_int32_T *iwork;
  int b_i;
  int q;
  emxArray_boolean_T *idelete;
  int qEnd;
  int kEnd;
  emxArray_boolean_T *r;
  emxArray_int32_T *r1;
  emxInit_int32_T(&idx, 1);
  emxInit_int32_T(&iInflect, 1);
  emxInit_int32_T(&b_sortIdx, 1);
  i = b_sortIdx->size[0];
  b_sortIdx->size[0] = Yin->size[1];
  emxEnsureCapacity_int32_T(b_sortIdx, i);
  i = iInflect->size[0];
  iInflect->size[0] = Yin->size[1];
  emxEnsureCapacity_int32_T(iInflect, i);
  ny = Yin->size[1];
  nPk = 0;
  nInflect = -1;
  dir = 'n';
  kfirst = 0;
  ykfirst = rtInf;
  isinfykfirst = true;
  for (k = 1; k <= ny; k++) {
    yk = Yin->data[k - 1];
    if (rtIsNaN(yk)) {
      yk = rtInf;
      isinfyk = true;
    } else {
      isinfyk = false;
    }

    if (yk != ykfirst) {
      previousdir = dir;
      if (isinfyk || isinfykfirst) {
        dir = 'n';
        if (kfirst >= 1) {
          nInflect++;
          iInflect->data[nInflect] = kfirst;
        }
      } else if (yk < ykfirst) {
        dir = 'd';
        if ('d' != previousdir) {
          nInflect++;
          iInflect->data[nInflect] = kfirst;
          if (previousdir == 'i') {
            nPk++;
            b_sortIdx->data[nPk - 1] = kfirst;
          }
        }
      } else {
        dir = 'i';
        if ('i' != previousdir) {
          nInflect++;
          iInflect->data[nInflect] = kfirst;
        }
      }

      ykfirst = yk;
      kfirst = k;
      isinfykfirst = isinfyk;
    }
  }

  if ((Yin->size[1] > 0) && (!isinfykfirst) && ((nInflect + 1 == 0) ||
       (iInflect->data[nInflect] < Yin->size[1]))) {
    nInflect++;
    iInflect->data[nInflect] = Yin->size[1];
  }

  emxInit_int32_T(&iPk, 1);
  if (1 > nPk) {
    i = 0;
  } else {
    i = nPk;
  }

  ny = b_sortIdx->size[0];
  b_sortIdx->size[0] = i;
  emxEnsureCapacity_int32_T(b_sortIdx, ny);
  idx->size[0] = 0;
  ny = iInflect->size[0];
  if (1 > nInflect + 1) {
    nInflect = -1;
  }

  iInflect->size[0] = nInflect + 1;
  emxEnsureCapacity_int32_T(iInflect, ny);
  ny = iPk->size[0];
  iPk->size[0] = i;
  emxEnsureCapacity_int32_T(iPk, ny);
  nPk = 0;
  for (k = 0; k < i; k++) {
    yk = Yin->data[b_sortIdx->data[k]];
    ykfirst = Yin->data[b_sortIdx->data[k] - 2];
    if ((ykfirst > yk) || rtIsNaN(yk)) {
      yk = ykfirst;
    }

    if (Yin->data[b_sortIdx->data[k] - 1] - yk >= 0.0) {
      nPk++;
      iPk->data[nPk - 1] = b_sortIdx->data[k];
    }
  }

  emxInit_int32_T(&b_iPk, 1);
  if (1 > nPk) {
    kfirst = 0;
  } else {
    kfirst = nPk;
  }

  i = iPk->size[0];
  if (1 > nPk) {
    iPk->size[0] = 0;
  } else {
    iPk->size[0] = nPk;
  }

  emxEnsureCapacity_int32_T(iPk, i);
  i = b_iPk->size[0];
  b_iPk->size[0] = kfirst;
  emxEnsureCapacity_int32_T(b_iPk, i);
  for (i = 0; i < kfirst; i++) {
    b_iPk->data[i] = iPk->data[i];
  }

  emxInit_real_T(&bxPk, 2);
  emxInit_real_T(&byPk, 2);
  emxInit_real_T(&wxPk, 2);
  emxInit_real_T(&locs_temp, 1);
  ny = Yin->size[1];
  kfirst = varargin_1->size[1];
  b_Yin = *Yin;
  c_Yin[0] = ny;
  b_Yin.size = &c_Yin[0];
  b_Yin.numDimensions = 1;
  b_varargin_1 = *varargin_1;
  c_varargin_1[0] = kfirst;
  b_varargin_1.size = &c_varargin_1[0];
  b_varargin_1.numDimensions = 1;
  findExtents(&b_Yin, &b_varargin_1, b_iPk, b_sortIdx, idx, iInflect, locs_temp,
              bxPk, byPk, wxPk);
  emxFree_real_T(&wxPk);
  emxFree_real_T(&byPk);
  emxFree_real_T(&bxPk);
  emxFree_int32_T(&iInflect);
  if (b_iPk->size[0] == 0) {
    idx->size[0] = 0;
  } else {
    n = b_iPk->size[0] + 1;
    i = b_sortIdx->size[0];
    b_sortIdx->size[0] = b_iPk->size[0];
    emxEnsureCapacity_int32_T(b_sortIdx, i);
    kfirst = b_iPk->size[0];
    for (i = 0; i < kfirst; i++) {
      b_sortIdx->data[i] = 0;
    }

    emxInit_int32_T(&iwork, 1);
    i = iwork->size[0];
    iwork->size[0] = b_iPk->size[0];
    emxEnsureCapacity_int32_T(iwork, i);
    i = b_iPk->size[0] - 1;
    for (k = 1; k <= i; k += 2) {
      ykfirst = Yin->data[b_iPk->data[k - 1] - 1];
      if ((ykfirst >= Yin->data[b_iPk->data[k] - 1]) || rtIsNaN(ykfirst)) {
        b_sortIdx->data[k - 1] = k;
        b_sortIdx->data[k] = k + 1;
      } else {
        b_sortIdx->data[k - 1] = k + 1;
        b_sortIdx->data[k] = k;
      }
    }

    if ((b_iPk->size[0] & 1) != 0) {
      b_sortIdx->data[b_iPk->size[0] - 1] = b_iPk->size[0];
    }

    b_i = 2;
    while (b_i < n - 1) {
      ny = b_i << 1;
      kfirst = 1;
      for (nPk = b_i + 1; nPk < n; nPk = qEnd + b_i) {
        nInflect = kfirst - 1;
        q = nPk;
        qEnd = kfirst + ny;
        if (qEnd > n) {
          qEnd = n;
        }

        k = 0;
        kEnd = qEnd - kfirst;
        while (k + 1 <= kEnd) {
          ykfirst = Yin->data[b_iPk->data[b_sortIdx->data[nInflect] - 1] - 1];
          i = b_sortIdx->data[q - 1];
          if ((ykfirst >= Yin->data[b_iPk->data[i - 1] - 1]) || rtIsNaN(ykfirst))
          {
            iwork->data[k] = b_sortIdx->data[nInflect];
            nInflect++;
            if (nInflect + 1 == nPk) {
              while (q < qEnd) {
                k++;
                iwork->data[k] = b_sortIdx->data[q - 1];
                q++;
              }
            }
          } else {
            iwork->data[k] = i;
            q++;
            if (q == qEnd) {
              while (nInflect + 1 < nPk) {
                k++;
                iwork->data[k] = b_sortIdx->data[nInflect];
                nInflect++;
              }
            }
          }

          k++;
        }

        for (k = 0; k < kEnd; k++) {
          b_sortIdx->data[(kfirst + k) - 1] = iwork->data[k];
        }

        kfirst = qEnd;
      }

      b_i = ny;
    }

    emxFree_int32_T(&iwork);
    i = locs_temp->size[0];
    locs_temp->size[0] = b_sortIdx->size[0];
    emxEnsureCapacity_real_T(locs_temp, i);
    kfirst = b_sortIdx->size[0];
    for (i = 0; i < kfirst; i++) {
      locs_temp->data[i] = varargin_1->data[b_iPk->data[b_sortIdx->data[i] - 1]
        - 1];
    }

    emxInit_boolean_T(&idelete, 1);
    i = idelete->size[0];
    idelete->size[0] = b_sortIdx->size[0];
    emxEnsureCapacity_boolean_T(idelete, i);
    kfirst = b_sortIdx->size[0];
    for (i = 0; i < kfirst; i++) {
      idelete->data[i] = false;
    }

    i = b_sortIdx->size[0];
    emxInit_boolean_T(&r, 1);
    for (b_i = 0; b_i < i; b_i++) {
      if (!idelete->data[b_i]) {
        ykfirst = varargin_1->data[b_iPk->data[b_sortIdx->data[b_i] - 1] - 1];
        ny = r->size[0];
        r->size[0] = locs_temp->size[0];
        emxEnsureCapacity_boolean_T(r, ny);
        kfirst = locs_temp->size[0];
        for (ny = 0; ny < kfirst; ny++) {
          r->data[ny] = ((locs_temp->data[ny] >= ykfirst - 50.0) &&
                         (locs_temp->data[ny] <= ykfirst + 50.0));
        }

        kfirst = idelete->size[0];
        for (ny = 0; ny < kfirst; ny++) {
          idelete->data[ny] = (idelete->data[ny] || r->data[ny]);
        }

        idelete->data[b_i] = false;
      }
    }

    emxFree_boolean_T(&r);
    kfirst = idelete->size[0] - 1;
    ny = 0;
    for (b_i = 0; b_i <= kfirst; b_i++) {
      if (!idelete->data[b_i]) {
        ny++;
      }
    }

    emxInit_int32_T(&r1, 1);
    i = r1->size[0];
    r1->size[0] = ny;
    emxEnsureCapacity_int32_T(r1, i);
    ny = 0;
    for (b_i = 0; b_i <= kfirst; b_i++) {
      if (!idelete->data[b_i]) {
        r1->data[ny] = b_i + 1;
        ny++;
      }
    }

    emxFree_boolean_T(&idelete);
    i = idx->size[0];
    idx->size[0] = r1->size[0];
    emxEnsureCapacity_int32_T(idx, i);
    kfirst = r1->size[0];
    for (i = 0; i < kfirst; i++) {
      idx->data[i] = b_sortIdx->data[r1->data[i] - 1];
    }

    emxFree_int32_T(&r1);
    sort(idx);
  }

  emxFree_real_T(&locs_temp);
  emxFree_int32_T(&b_sortIdx);
  if (idx->size[0] > Yin->size[1]) {
    i = idx->size[0];
    idx->size[0] = Yin->size[1];
    emxEnsureCapacity_int32_T(idx, i);
  }

  i = iPk->size[0];
  iPk->size[0] = idx->size[0];
  emxEnsureCapacity_int32_T(iPk, i);
  kfirst = idx->size[0];
  for (i = 0; i < kfirst; i++) {
    iPk->data[i] = b_iPk->data[idx->data[i] - 1];
  }

  emxFree_int32_T(&b_iPk);
  emxFree_int32_T(&idx);
  i = Ypk->size[0] * Ypk->size[1];
  Ypk->size[0] = 1;
  Ypk->size[1] = iPk->size[0];
  emxEnsureCapacity_real_T(Ypk, i);
  kfirst = iPk->size[0];
  for (i = 0; i < kfirst; i++) {
    Ypk->data[i] = Yin->data[iPk->data[i] - 1];
  }

  i = Xpk->size[0] * Xpk->size[1];
  Xpk->size[0] = 1;
  Xpk->size[1] = iPk->size[0];
  emxEnsureCapacity_real_T(Xpk, i);
  kfirst = iPk->size[0];
  for (i = 0; i < kfirst; i++) {
    Xpk->data[i] = varargin_1->data[iPk->data[i] - 1];
  }

  emxFree_int32_T(&iPk);
}

/* End of code generation (findpeaks.cpp) */
