/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * histcounts2.cpp
 *
 * Code generation for function 'histcounts2'
 *
 */

/* Include files */
#include "histcounts2.h"
#include "generateGlobalFeature.h"
#include "mapElementsToBins.h"
#include "rt_nonfinite.h"
#include <cstring>

/* Function Definitions */
void histcounts2(const emxArray_real_T *x, const emxArray_real_T *y, const
                 double varargin_2[9], double n[128])
{
  static const double varargin_1[17] = { -3.1415926535897931, -2.748893571891069,
    -2.3561944901923448, -1.9634954084936207, -1.5707963267948966,
    -1.1780972450961724, -0.78539816339744828, -0.39269908169872414, 0.0,
    0.39269908169872414, 0.78539816339744828, 1.1780972450961724,
    1.5707963267948966, 1.9634954084936207, 2.3561944901923448,
    2.748893571891069, 3.1415926535897931 };

  int unusedU1[16];
  int binx_data[10000];
  int binx_size[2];
  int unusedU2[8];
  int biny_data[10000];
  int biny_size[2];
  int ni[128];
  int nb;
  int k;
  int ni_tmp;
  mapElementsToBins(x, varargin_1, unusedU1, binx_data, binx_size);
  b_mapElementsToBins(y, varargin_2, unusedU2, biny_data, biny_size);
  std::memset(&ni[0], 0, 128U * sizeof(int));
  nb = binx_size[1];
  for (k = 0; k < nb; k++) {
    if ((binx_data[k] != 0) && (biny_data[k] != 0)) {
      ni_tmp = (binx_data[k] + ((biny_data[k] - 1) << 4)) - 1;
      ni[ni_tmp]++;
    } else {
      binx_data[k] = 0;
      biny_data[k] = 0;
    }
  }

  for (nb = 0; nb < 128; nb++) {
    n[nb] = ni[nb];
  }
}

/* End of code generation (histcounts2.cpp) */
