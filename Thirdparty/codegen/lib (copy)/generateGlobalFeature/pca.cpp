/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * pca.cpp
 *
 * Code generation for function 'pca'
 *
 */

/* Include files */
#include "pca.h"
#include "generateGlobalFeature.h"
#include "generateGlobalFeature_emxutil.h"
#include "rt_nonfinite.h"
#include "xzsvdc.h"
#include <cmath>
#include <cstring>
#include <math.h>

/* Function Declarations */
static void b_localSVD(emxArray_real_T *x, int DOF, const emxArray_real_T
  *Weights, double coeffOut_data[], int coeffOut_size[2], emxArray_real_T
  *scoreOut, double latentOut_data[], int latentOut_size[1], emxArray_real_T
  *tsquared, double explained_data[], int explained_size[1]);
static void localSVD(const emxArray_real_T *x, int DOF, double coeffOut_data[],
                     int coeffOut_size[2], emxArray_real_T *scoreOut, double
                     latentOut_data[], int latentOut_size[1], emxArray_real_T
                     *tsquared, double explained_data[], int explained_size[1]);
static void localTSquared(const emxArray_real_T *score, const double
  latent_data[], const int latent_size[1], int DOF, emxArray_real_T *tsquared);

/* Function Definitions */
static void b_localSVD(emxArray_real_T *x, int DOF, const emxArray_real_T
  *Weights, double coeffOut_data[], int coeffOut_size[2], emxArray_real_T
  *scoreOut, double latentOut_data[], int latentOut_size[1], emxArray_real_T
  *tsquared, double explained_data[], int explained_size[1])
{
  emxArray_real_T *OmegaSqrt;
  int k;
  int nx;
  int nrows;
  int j;
  emxArray_real_T *b_x;
  emxArray_real_T *score;
  double latent_data[3];
  int latent_size[1];
  double coeff_data[9];
  int coeff_size[2];
  int nsv;
  double y;
  emxInit_real_T(&OmegaSqrt, 1);
  k = OmegaSqrt->size[0];
  OmegaSqrt->size[0] = Weights->size[0];
  emxEnsureCapacity_real_T(OmegaSqrt, k);
  nx = Weights->size[0];
  for (k = 0; k < nx; k++) {
    OmegaSqrt->data[k] = Weights->data[k];
  }

  nx = Weights->size[0];
  for (k = 0; k < nx; k++) {
    OmegaSqrt->data[k] = std::sqrt(OmegaSqrt->data[k]);
  }

  nrows = x->size[0] - 1;
  for (j = 0; j < 3; j++) {
    for (nx = 0; nx <= nrows; nx++) {
      x->data[nx + x->size[0] * j] *= OmegaSqrt->data[nx];
    }
  }

  emxInit_real_T(&b_x, 2);
  k = b_x->size[0] * b_x->size[1];
  b_x->size[0] = x->size[0];
  b_x->size[1] = 3;
  emxEnsureCapacity_real_T(b_x, k);
  nx = x->size[0] * x->size[1] - 1;
  for (k = 0; k <= nx; k++) {
    b_x->data[k] = x->data[k];
  }

  emxInit_real_T(&score, 2);
  xzsvdc(b_x, score, latent_data, latent_size, coeff_data, coeff_size);
  nsv = score->size[1];
  emxFree_real_T(&b_x);
  for (j = 0; j < nsv; j++) {
    for (nx = 0; nx <= nrows; nx++) {
      score->data[nx + score->size[0] * j] = score->data[nx + score->size[0] * j]
        / OmegaSqrt->data[nx] * latent_data[j];
    }
  }

  emxFree_real_T(&OmegaSqrt);
  for (j = 0; j < nsv; j++) {
    latent_data[j] = latent_data[j] * latent_data[j] / static_cast<double>(DOF);
  }

  localTSquared(score, latent_data, latent_size, DOF, tsquared);
  if (DOF < 3) {
    if (DOF < nsv) {
      nsv = DOF;
    }

    k = scoreOut->size[0] * scoreOut->size[1];
    scoreOut->size[0] = nrows + 1;
    scoreOut->size[1] = nsv;
    emxEnsureCapacity_real_T(scoreOut, k);
    latentOut_size[0] = nsv;
    coeffOut_size[0] = 3;
    coeffOut_size[1] = nsv;
    for (j = 0; j < nsv; j++) {
      for (nx = 0; nx <= nrows; nx++) {
        scoreOut->data[nx + scoreOut->size[0] * j] = score->data[nx +
          score->size[0] * j];
      }

      latentOut_data[j] = latent_data[j];
      coeffOut_data[3 * j] = coeff_data[3 * j];
      k = 3 * j + 1;
      coeffOut_data[k] = coeff_data[k];
      k = 3 * j + 2;
      coeffOut_data[k] = coeff_data[k];
    }
  } else {
    k = scoreOut->size[0] * scoreOut->size[1];
    scoreOut->size[0] = score->size[0];
    scoreOut->size[1] = score->size[1];
    emxEnsureCapacity_real_T(scoreOut, k);
    nx = score->size[0] * score->size[1];
    for (k = 0; k < nx; k++) {
      scoreOut->data[k] = score->data[k];
    }

    latentOut_size[0] = latent_size[0];
    nx = latent_size[0];
    if (0 <= nx - 1) {
      std::memcpy(&latentOut_data[0], &latent_data[0], nx * sizeof(double));
    }

    coeffOut_size[0] = 3;
    coeffOut_size[1] = coeff_size[1];
    nx = coeff_size[0] * coeff_size[1];
    if (0 <= nx - 1) {
      std::memcpy(&coeffOut_data[0], &coeff_data[0], nx * sizeof(double));
    }
  }

  emxFree_real_T(&score);
  nx = latentOut_size[0];
  if (latentOut_size[0] == 0) {
    y = 0.0;
  } else {
    y = latentOut_data[0];
    for (k = 2; k <= nx; k++) {
      y += latentOut_data[k - 1];
    }
  }

  explained_size[0] = latentOut_size[0];
  nx = latentOut_size[0];
  for (k = 0; k < nx; k++) {
    explained_data[k] = 100.0 * latentOut_data[k] / y;
  }
}

static void localSVD(const emxArray_real_T *x, int DOF, double coeffOut_data[],
                     int coeffOut_size[2], emxArray_real_T *scoreOut, double
                     latentOut_data[], int latentOut_size[1], emxArray_real_T
                     *tsquared, double explained_data[], int explained_size[1])
{
  emxArray_real_T *b_x;
  int nrows;
  int k;
  int vlen;
  emxArray_real_T *score;
  double latent_data[3];
  int latent_size[1];
  double coeff_data[9];
  int coeff_size[2];
  int nsv;
  int j;
  emxArray_real_T *r;
  double y;
  emxInit_real_T(&b_x, 2);
  nrows = x->size[0];
  k = b_x->size[0] * b_x->size[1];
  b_x->size[0] = x->size[0];
  b_x->size[1] = 3;
  emxEnsureCapacity_real_T(b_x, k);
  vlen = x->size[0] * x->size[1] - 1;
  for (k = 0; k <= vlen; k++) {
    b_x->data[k] = x->data[k];
  }

  emxInit_real_T(&score, 2);
  xzsvdc(b_x, score, latent_data, latent_size, coeff_data, coeff_size);
  nsv = score->size[1];
  emxFree_real_T(&b_x);
  for (j = 0; j < nsv; j++) {
    for (vlen = 0; vlen < nrows; vlen++) {
      score->data[vlen + score->size[0] * j] *= latent_data[j];
    }

    latent_data[j] = latent_data[j] * latent_data[j] / static_cast<double>(DOF);
  }

  emxInit_real_T(&r, 2);
  localTSquared(score, latent_data, latent_size, DOF, r);
  k = tsquared->size[0] * tsquared->size[1];
  tsquared->size[0] = r->size[0];
  tsquared->size[1] = r->size[1];
  emxEnsureCapacity_real_T(tsquared, k);
  vlen = r->size[0] * r->size[1];
  for (k = 0; k < vlen; k++) {
    tsquared->data[k] = r->data[k];
  }

  emxFree_real_T(&r);
  if (DOF < 3) {
    if (DOF < nsv) {
      nsv = DOF;
    }

    k = scoreOut->size[0] * scoreOut->size[1];
    scoreOut->size[0] = x->size[0];
    scoreOut->size[1] = nsv;
    emxEnsureCapacity_real_T(scoreOut, k);
    latentOut_size[0] = nsv;
    coeffOut_size[0] = 3;
    coeffOut_size[1] = nsv;
    for (j = 0; j < nsv; j++) {
      for (vlen = 0; vlen < nrows; vlen++) {
        scoreOut->data[vlen + scoreOut->size[0] * j] = score->data[vlen +
          score->size[0] * j];
      }

      latentOut_data[j] = latent_data[j];
      coeffOut_data[3 * j] = coeff_data[3 * j];
      k = 3 * j + 1;
      coeffOut_data[k] = coeff_data[k];
      k = 3 * j + 2;
      coeffOut_data[k] = coeff_data[k];
    }
  } else {
    k = scoreOut->size[0] * scoreOut->size[1];
    scoreOut->size[0] = score->size[0];
    scoreOut->size[1] = score->size[1];
    emxEnsureCapacity_real_T(scoreOut, k);
    vlen = score->size[0] * score->size[1];
    for (k = 0; k < vlen; k++) {
      scoreOut->data[k] = score->data[k];
    }

    latentOut_size[0] = latent_size[0];
    vlen = latent_size[0];
    if (0 <= vlen - 1) {
      std::memcpy(&latentOut_data[0], &latent_data[0], vlen * sizeof(double));
    }

    coeffOut_size[0] = 3;
    coeffOut_size[1] = coeff_size[1];
    vlen = coeff_size[0] * coeff_size[1];
    if (0 <= vlen - 1) {
      std::memcpy(&coeffOut_data[0], &coeff_data[0], vlen * sizeof(double));
    }
  }

  emxFree_real_T(&score);
  vlen = latentOut_size[0];
  if (latentOut_size[0] == 0) {
    y = 0.0;
  } else {
    y = latentOut_data[0];
    for (k = 2; k <= vlen; k++) {
      y += latentOut_data[k - 1];
    }
  }

  explained_size[0] = latentOut_size[0];
  vlen = latentOut_size[0];
  for (k = 0; k < vlen; k++) {
    explained_data[k] = 100.0 * latentOut_data[k] / y;
  }
}

static void localTSquared(const emxArray_real_T *score, const double
  latent_data[], const int latent_size[1], int DOF, emxArray_real_T *tsquared)
{
  int i;
  int q;
  double absx;
  int m;
  int exponent;
  double d;
  if ((score->size[0] == 0) || (score->size[1] == 0)) {
    i = tsquared->size[0] * tsquared->size[1];
    tsquared->size[0] = score->size[0];
    tsquared->size[1] = score->size[1];
    emxEnsureCapacity_real_T(tsquared, i);
    exponent = score->size[0] * score->size[1];
    for (i = 0; i < exponent; i++) {
      tsquared->data[i] = score->data[i];
    }
  } else {
    if (DOF > 1) {
      absx = std::abs(latent_data[0]);
      if ((!rtIsInf(absx)) && (!rtIsNaN(absx))) {
        if (absx <= 2.2250738585072014E-308) {
          absx = 4.94065645841247E-324;
        } else {
          frexp(absx, &exponent);
          absx = std::ldexp(1.0, exponent - 53);
        }
      } else {
        absx = rtNaN;
      }

      if (DOF > 3) {
        exponent = DOF;
      } else {
        exponent = 3;
      }

      absx *= static_cast<double>(exponent);
      q = 0;
      i = latent_size[0];
      for (exponent = 0; exponent < i; exponent++) {
        if (latent_data[exponent] > absx) {
          q++;
        }
      }
    } else {
      q = 0;
    }

    m = score->size[0];
    i = tsquared->size[0] * tsquared->size[1];
    tsquared->size[0] = score->size[0];
    tsquared->size[1] = 1;
    emxEnsureCapacity_real_T(tsquared, i);
    exponent = score->size[0];
    for (i = 0; i < exponent; i++) {
      tsquared->data[i] = 0.0;
    }

    for (exponent = 0; exponent < q; exponent++) {
      absx = std::sqrt(latent_data[exponent]);
      for (i = 0; i < m; i++) {
        d = score->data[i + score->size[0] * exponent] / absx;
        tsquared->data[i] += d * d;
      }
    }
  }
}

void local_pca(emxArray_real_T *x, double coeffOut_data[], int coeffOut_size[2])
{
  int n;
  int b_n;
  int naninfo_nNaNs;
  int naninfo_nRowsWithNaNs;
  int DOF;
  int naninfo_nNaNsInRow_data[10000];
  int naninfo_isNaN_size_idx_0;
  int i;
  int j;
  boolean_T naninfo_isNaN_data[30000];
  boolean_T noNaNs;
  double wcol;
  double xcol;
  emxArray_real_T *y;
  double mu[3];
  double d;
  emxArray_real_T *xNoNaNs;
  emxArray_real_T *tsquared;
  emxArray_real_T *WNoNaNs;
  double coeff_data[9];
  int coeff_size[2];
  double latent_data[3];
  int latent_size[1];
  double explained_data[3];
  int explained_size[1];
  emxArray_real_T *b_tsquared;
  double sgn;
  double d1;
  n = x->size[0];
  b_n = x->size[0] - 1;
  naninfo_nNaNs = 0;
  naninfo_nRowsWithNaNs = 0;
  DOF = x->size[0];
  if (0 <= DOF - 1) {
    std::memset(&naninfo_nNaNsInRow_data[0], 0, DOF * sizeof(int));
  }

  naninfo_isNaN_size_idx_0 = x->size[0];
  DOF = x->size[0] * x->size[1];
  for (i = 0; i < DOF; i++) {
    naninfo_isNaN_data[i] = rtIsNaN(x->data[i]);
  }

  for (j = 0; j < 3; j++) {
    for (i = 0; i <= b_n; i++) {
      if (naninfo_isNaN_data[i + naninfo_isNaN_size_idx_0 * j]) {
        naninfo_nNaNsInRow_data[i]++;
        naninfo_nNaNs++;
      }
    }
  }

  for (i = 0; i <= b_n; i++) {
    if (naninfo_nNaNsInRow_data[i] > 0) {
      naninfo_nRowsWithNaNs++;
    }
  }

  noNaNs = (naninfo_nNaNs <= 0);
  DOF = x->size[0] - naninfo_nRowsWithNaNs;
  if (DOF >= 1) {
    DOF--;
  }

  b_n = x->size[0] - 1;
  if (!noNaNs) {
    for (j = 0; j < 3; j++) {
      wcol = 0.0;
      xcol = 0.0;
      for (i = 0; i <= b_n; i++) {
        d = x->data[i + x->size[0] * j];
        if (!rtIsNaN(d)) {
          wcol++;
          xcol += d;
        }
      }

      mu[j] = xcol / wcol;
    }
  } else {
    for (j = 0; j < 3; j++) {
      wcol = 0.0;
      xcol = 0.0;
      for (i = 0; i <= b_n; i++) {
        wcol++;
        xcol += x->data[i + x->size[0] * j];
      }

      mu[j] = xcol / wcol;
    }
  }

  for (j = 0; j < 3; j++) {
    for (i = 0; i < n; i++) {
      x->data[i + x->size[0] * j] -= mu[j];
    }
  }

  emxInit_real_T(&y, 2);
  if (noNaNs) {
    emxInit_real_T(&tsquared, 2);
    localSVD(x, DOF, coeff_data, coeff_size, y, latent_data, latent_size,
             tsquared, explained_data, explained_size);
    emxFree_real_T(&tsquared);
  } else {
    emxInit_real_T(&xNoNaNs, 2);
    emxInit_real_T(&WNoNaNs, 1);
    n = x->size[0];
    b_n = x->size[0] - naninfo_nRowsWithNaNs;
    i = xNoNaNs->size[0] * xNoNaNs->size[1];
    xNoNaNs->size[0] = b_n;
    xNoNaNs->size[1] = 3;
    emxEnsureCapacity_real_T(xNoNaNs, i);
    i = WNoNaNs->size[0];
    WNoNaNs->size[0] = b_n;
    emxEnsureCapacity_real_T(WNoNaNs, i);
    b_n = -1;
    for (i = 0; i < n; i++) {
      if (naninfo_nNaNsInRow_data[i] == 0) {
        b_n++;
        xNoNaNs->data[b_n] = x->data[i];
        xNoNaNs->data[b_n + xNoNaNs->size[0]] = x->data[i + x->size[0]];
        xNoNaNs->data[b_n + xNoNaNs->size[0] * 2] = x->data[i + x->size[0] * 2];
        WNoNaNs->data[b_n] = 1.0;
      }
    }

    emxInit_real_T(&b_tsquared, 2);
    b_localSVD(xNoNaNs, DOF, WNoNaNs, coeff_data, coeff_size, y, latent_data,
               latent_size, b_tsquared, explained_data, explained_size);
    emxFree_real_T(&b_tsquared);
    emxFree_real_T(&WNoNaNs);
    emxFree_real_T(&xNoNaNs);
  }

  emxFree_real_T(&y);
  if (3 < DOF) {
    coeffOut_size[0] = 3;
    coeffOut_size[1] = 3;
    for (j = 0; j < 3; j++) {
      coeffOut_data[3 * j] = coeff_data[3 * j];
      b_n = 3 * j + 1;
      coeffOut_data[b_n] = coeff_data[b_n];
      b_n = 3 * j + 2;
      coeffOut_data[b_n] = coeff_data[b_n];
    }
  } else {
    coeffOut_size[0] = 3;
    coeffOut_size[1] = coeff_size[1];
    DOF = coeff_size[0] * coeff_size[1];
    if (0 <= DOF - 1) {
      std::memcpy(&coeffOut_data[0], &coeff_data[0], DOF * sizeof(double));
    }
  }

  b_n = coeffOut_size[1];
  for (j = 0; j < b_n; j++) {
    xcol = 0.0;
    sgn = 1.0;
    d = coeffOut_data[3 * j];
    wcol = std::abs(d);
    if (wcol > 0.0) {
      xcol = wcol;
      sgn = d;
      if (d < 0.0) {
        sgn = -1.0;
      } else if (d > 0.0) {
        sgn = 1.0;
      } else {
        if (d == 0.0) {
          sgn = 0.0;
        }
      }
    }

    i = 3 * j + 1;
    d1 = coeffOut_data[i];
    wcol = std::abs(coeffOut_data[i]);
    if (wcol > xcol) {
      xcol = wcol;
      sgn = coeffOut_data[i];
      if (coeffOut_data[i] < 0.0) {
        sgn = -1.0;
      } else if (coeffOut_data[i] > 0.0) {
        sgn = 1.0;
      } else {
        if (coeffOut_data[i] == 0.0) {
          sgn = 0.0;
        }
      }
    }

    DOF = 3 * j + 2;
    wcol = coeffOut_data[DOF];
    if (std::abs(coeffOut_data[DOF]) > xcol) {
      sgn = coeffOut_data[DOF];
      if (coeffOut_data[DOF] < 0.0) {
        sgn = -1.0;
      } else if (coeffOut_data[DOF] > 0.0) {
        sgn = 1.0;
      } else {
        if (coeffOut_data[DOF] == 0.0) {
          sgn = 0.0;
        }
      }
    }

    if (sgn < 0.0) {
      d = -d;
      coeffOut_data[3 * j] = d;
      d1 = -d1;
      coeffOut_data[i] = d1;
      wcol = -wcol;
      coeffOut_data[DOF] = wcol;
    }
  }
}

/* End of code generation (pca.cpp) */
