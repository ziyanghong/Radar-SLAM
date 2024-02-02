/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * M2DP.cpp
 *
 * Code generation for function 'M2DP'
 *
 */

/* Include files */
#include "M2DP.h"
#include "cart2pol.h"
#include "generateGlobalFeature.h"
#include "generateGlobalFeature_emxutil.h"
#include "histcounts2.h"
#include "pca.h"
#include "repmat.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

/* Function Definitions */
void GetSignatureMatrix(const emxArray_real_T *data, double maxRho, double A
  [8192])
{
  int n;
  double delta1;
  double a[9];
  int k;
  emxArray_real_T *y;
  double rhoList[9];
  emxArray_real_T *b_y;
  emxArray_real_T *pdata;
  emxArray_real_T *b_pdata;
  emxArray_real_T *c_y;
  emxArray_real_T *d_y;
  int m;
  int b_m;
  emxArray_real_T *c_pdata;
  int p;
  double d;
  static const double dv[4] = { -1.5707963267948966, -0.52359877559829882,
    0.52359877559829882, 1.5707963267948966 };

  double d1;
  int q;
  double rcoselev_tmp;
  double vecN_idx_0;
  double vecN_idx_1;
  double px[3];
  int i;
  double b[3];
  int aoffset;
  double dv1[128];

  /*  signature matrix A */
  std::memset(&A[0], 0, 8192U * sizeof(double));

  /*  plane index */
  n = 0;

  /*  theta list */
  /*  rho list */
  delta1 = std::sqrt(maxRho);
  a[8] = delta1;
  a[0] = 0.0;
  if (0.0 == -delta1) {
    for (k = 0; k < 7; k++) {
      a[k + 1] = delta1 * ((2.0 * (static_cast<double>(k) + 2.0) - 10.0) / 8.0);
    }

    a[4] = 0.0;
  } else if ((delta1 < 0.0) && (std::abs(delta1) > 8.9884656743115785E+307)) {
    delta1 /= 8.0;
    for (k = 0; k < 7; k++) {
      a[k + 1] = delta1 * (static_cast<double>(k) + 1.0);
    }
  } else {
    delta1 /= 8.0;
    for (k = 0; k < 7; k++) {
      a[k + 1] = (static_cast<double>(k) + 1.0) * delta1;
    }
  }

  for (k = 0; k < 9; k++) {
    rhoList[k] = a[k] * a[k];
  }

  emxInit_real_T(&y, 1);
  emxInit_real_T(&b_y, 1);
  emxInit_real_T(&pdata, 1);
  emxInit_real_T(&b_pdata, 1);
  emxInit_real_T(&c_y, 2);
  emxInit_real_T(&d_y, 2);
  rhoList[8] += 0.001;

  /*  make sure all points in bins */
  /*  loop on azimuth */
  m = data->size[0];
  b_m = data->size[0];
  emxInit_real_T(&c_pdata, 2);
  for (p = 0; p < 4; p++) {
    /*  pick one azimuth */
    /*  loop on evevation */
    d = std::cos(dv[p]);
    d1 = std::sin(dv[p]);
    for (q = 0; q < 16; q++) {
      /*  pick one elevation */
      /*  normal vector vecN of the selected 2D plane */
      rcoselev_tmp = 0.10471975511965977 * static_cast<double>(q);
      delta1 = std::cos(rcoselev_tmp);
      vecN_idx_0 = delta1 * d;
      vecN_idx_1 = delta1 * d1;
      delta1 = std::sin(rcoselev_tmp);

      /*  distance of vector [1,0,0] to the surface with normal vector vecN */
      rcoselev_tmp = (vecN_idx_0 + 0.0 * vecN_idx_1) + 0.0 * delta1;

      /*  a new vector, c = h*vecN, so that vector [1,0,0]-c is the */
      /*  projection of x-axis onto the plane with normal vector vecN */
      /*  x-axis - c, the projection */
      px[0] = 1.0 - rcoselev_tmp * vecN_idx_0;
      px[1] = 0.0 - rcoselev_tmp * vecN_idx_1;
      px[2] = 0.0 - rcoselev_tmp * delta1;

      /*  given the normal vector vecN and the projected x-axis px, the */
      /*  y- axis is cross(vecN,px) */
      /*  projection of data onto space span{px,py} */
      k = y->size[0];
      y->size[0] = data->size[0];
      emxEnsureCapacity_real_T(y, k);
      for (i = 0; i < m; i++) {
        y->data[i] = 0.0;
      }

      for (k = 0; k < 3; k++) {
        aoffset = k * m;
        for (i = 0; i < m; i++) {
          y->data[i] += px[k] * data->data[aoffset + i];
        }
      }

      b[0] = vecN_idx_1 * px[2] - delta1 * px[1];
      b[1] = delta1 * px[0] - vecN_idx_0 * px[2];
      b[2] = vecN_idx_0 * px[1] - vecN_idx_1 * px[0];
      k = b_y->size[0];
      b_y->size[0] = data->size[0];
      emxEnsureCapacity_real_T(b_y, k);
      for (i = 0; i < b_m; i++) {
        b_y->data[i] = 0.0;
      }

      for (k = 0; k < 3; k++) {
        aoffset = k * b_m;
        for (i = 0; i < b_m; i++) {
          b_y->data[i] += b[k] * data->data[aoffset + i];
        }
      }

      i = y->size[0];
      k = c_pdata->size[0] * c_pdata->size[1];
      c_pdata->size[0] = y->size[0];
      c_pdata->size[1] = 2;
      emxEnsureCapacity_real_T(c_pdata, k);
      for (k = 0; k < i; k++) {
        c_pdata->data[k] = y->data[k];
      }

      i = b_y->size[0];
      for (k = 0; k < i; k++) {
        c_pdata->data[k + c_pdata->size[0]] = b_y->data[k];
      }

      /*  represent data in polar coordinates */
      i = c_pdata->size[0];
      k = pdata->size[0];
      pdata->size[0] = c_pdata->size[0];
      emxEnsureCapacity_real_T(pdata, k);
      for (k = 0; k < i; k++) {
        pdata->data[k] = c_pdata->data[k];
      }

      i = c_pdata->size[0];
      k = b_pdata->size[0];
      b_pdata->size[0] = c_pdata->size[0];
      emxEnsureCapacity_real_T(b_pdata, k);
      for (k = 0; k < i; k++) {
        b_pdata->data[k] = c_pdata->data[k + c_pdata->size[0]];
      }

      cart2pol(pdata, b_pdata, y, b_y);

      /*  main function, count points in bins, use eigher a) C codes or b) */
      /*  Matlab built-in function */
      /*  a) C codes */
      /*          bin = CountPoint(theta, thetaList, rho, rhoList); */
      /*          % b) Matlab codes */
      /*  record the sigature of the n-th plane */
      k = c_y->size[0] * c_y->size[1];
      c_y->size[0] = 1;
      i = y->size[0];
      c_y->size[1] = y->size[0];
      emxEnsureCapacity_real_T(c_y, k);
      for (k = 0; k < i; k++) {
        c_y->data[k] = y->data[k];
      }

      k = d_y->size[0] * d_y->size[1];
      d_y->size[0] = 1;
      i = b_y->size[0];
      d_y->size[1] = b_y->size[0];
      emxEnsureCapacity_real_T(d_y, k);
      for (k = 0; k < i; k++) {
        d_y->data[k] = b_y->data[k];
      }

      histcounts2(c_y, d_y, rhoList, dv1);
      for (k = 0; k < 128; k++) {
        delta1 = dv1[k] / static_cast<double>(data->size[0]);
        dv1[k] = delta1;
        A[n + (k << 6)] = delta1;
      }

      n++;
    }
  }

  emxFree_real_T(&d_y);
  emxFree_real_T(&c_y);
  emxFree_real_T(&b_pdata);
  emxFree_real_T(&pdata);
  emxFree_real_T(&b_y);
  emxFree_real_T(&y);
  emxFree_real_T(&c_pdata);
}

void PCARotationInvariant(emxArray_real_T *data)
{
  int vlen;
  int boffset;
  int i;
  emxArray_real_T *b_data;
  int xpageoffset;
  double y[3];
  int k;
  emxArray_real_T *b;
  double pc_data[9];
  int pc_size[2];
  emxArray_real_T *X;
  emxArray_real_T *Y;
  emxArray_real_T *Z;

  /*  3D rotate input data so that x-axis and y-axis are the 1st and 2nd PCs of */
  /*  data, respectively */
  /*  shift data so that origin is the mean of data */
  /*  mean of data */
  vlen = data->size[0];
  boffset = data->size[0];
  for (i = 0; i < 3; i++) {
    xpageoffset = i * data->size[0];
    y[i] = data->data[xpageoffset];
    for (k = 2; k <= vlen; k++) {
      y[i] += data->data[(xpageoffset + k) - 1];
    }

    y[i] /= static_cast<double>(boffset);
  }

  emxInit_real_T(&b_data, 2);
  repmat(y, static_cast<double>(data->size[0]), b_data);
  i = data->size[0] * data->size[1];
  data->size[1] = 3;
  emxEnsureCapacity_real_T(data, i);
  for (i = 0; i < 3; i++) {
    vlen = data->size[0];
    for (boffset = 0; boffset < vlen; boffset++) {
      data->data[boffset + data->size[0] * i] -= b_data->data[boffset +
        b_data->size[0] * i];
    }
  }

  i = b_data->size[0] * b_data->size[1];
  b_data->size[0] = data->size[0];
  b_data->size[1] = 3;
  emxEnsureCapacity_real_T(b_data, i);
  vlen = data->size[0] * data->size[1] - 1;
  for (i = 0; i <= vlen; i++) {
    b_data->data[i] = data->data[i];
  }

  emxInit_real_T(&b, 2);
  local_pca(b_data, pc_data, pc_size);
  i = b->size[0] * b->size[1];
  b->size[0] = 3;
  b->size[1] = data->size[0];
  emxEnsureCapacity_real_T(b, i);
  vlen = data->size[0];
  emxFree_real_T(&b_data);
  for (i = 0; i < vlen; i++) {
    b->data[3 * i] = data->data[i];
    b->data[3 * i + 1] = data->data[i + data->size[0]];
    b->data[3 * i + 2] = data->data[i + data->size[0] * 2];
  }

  emxInit_real_T(&X, 2);
  xpageoffset = b->size[1];
  i = X->size[0] * X->size[1];
  X->size[0] = 1;
  X->size[1] = b->size[1];
  emxEnsureCapacity_real_T(X, i);
  for (vlen = 0; vlen < xpageoffset; vlen++) {
    boffset = vlen * 3;
    X->data[vlen] = (b->data[boffset] * pc_data[0] + b->data[boffset + 1] *
                     pc_data[1]) + b->data[boffset + 2] * pc_data[2];
  }

  i = b->size[0] * b->size[1];
  b->size[0] = 3;
  b->size[1] = data->size[0];
  emxEnsureCapacity_real_T(b, i);
  vlen = data->size[0];
  for (i = 0; i < vlen; i++) {
    b->data[3 * i] = data->data[i];
    b->data[3 * i + 1] = data->data[i + data->size[0]];
    b->data[3 * i + 2] = data->data[i + data->size[0] * 2];
  }

  emxInit_real_T(&Y, 2);
  xpageoffset = b->size[1];
  i = Y->size[0] * Y->size[1];
  Y->size[0] = 1;
  Y->size[1] = b->size[1];
  emxEnsureCapacity_real_T(Y, i);
  for (vlen = 0; vlen < xpageoffset; vlen++) {
    boffset = vlen * 3;
    Y->data[vlen] = (b->data[boffset] * pc_data[3] + b->data[boffset + 1] *
                     pc_data[4]) + b->data[boffset + 2] * pc_data[5];
  }

  i = b->size[0] * b->size[1];
  b->size[0] = 3;
  b->size[1] = data->size[0];
  emxEnsureCapacity_real_T(b, i);
  vlen = data->size[0];
  for (i = 0; i < vlen; i++) {
    b->data[3 * i] = data->data[i];
    b->data[3 * i + 1] = data->data[i + data->size[0]];
    b->data[3 * i + 2] = data->data[i + data->size[0] * 2];
  }

  emxInit_real_T(&Z, 2);
  xpageoffset = b->size[1];
  i = Z->size[0] * Z->size[1];
  Z->size[0] = 1;
  Z->size[1] = b->size[1];
  emxEnsureCapacity_real_T(Z, i);
  for (vlen = 0; vlen < xpageoffset; vlen++) {
    boffset = vlen * 3;
    Z->data[vlen] = (b->data[boffset] * pc_data[6] + b->data[boffset + 1] *
                     pc_data[7]) + b->data[boffset + 2] * pc_data[8];
  }

  emxFree_real_T(&b);
  i = data->size[0] * data->size[1];
  data->size[0] = X->size[1];
  data->size[1] = 3;
  emxEnsureCapacity_real_T(data, i);
  vlen = X->size[1];
  for (i = 0; i < vlen; i++) {
    data->data[i] = X->data[i];
  }

  emxFree_real_T(&X);
  vlen = Y->size[1];
  for (i = 0; i < vlen; i++) {
    data->data[i + data->size[0]] = Y->data[i];
  }

  emxFree_real_T(&Y);
  vlen = Z->size[1];
  for (i = 0; i < vlen; i++) {
    data->data[i + data->size[0] * 2] = Z->data[i];
  }

  emxFree_real_T(&Z);
}

/* End of code generation (M2DP.cpp) */
