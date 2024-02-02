/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * mapElementsToBins.cpp
 *
 * Code generation for function 'mapElementsToBins'
 *
 */

/* Include files */
#include "mapElementsToBins.h"
#include "generateGlobalFeature.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

/* Function Definitions */
void b_mapElementsToBins(const emxArray_real_T *x, const double edges[9], int n
  [8], int bin_data[], int bin_size[2])
{
  int low_i;
  short unnamed_idx_1;
  int nx;
  double leftEdge;
  double delta;
  int k;
  double d;
  double bGuess;
  boolean_T guard1 = false;
  int low_ip1;
  int high_i;
  int mid_i;
  for (low_i = 0; low_i < 8; low_i++) {
    n[low_i] = 0;
  }

  unnamed_idx_1 = static_cast<short>(x->size[1]);
  bin_size[0] = 1;
  bin_size[1] = unnamed_idx_1;
  low_i = unnamed_idx_1;
  if (0 <= low_i - 1) {
    std::memset(&bin_data[0], 0, low_i * sizeof(int));
  }

  nx = x->size[1];
  leftEdge = edges[0];
  delta = edges[1] - edges[0];
  for (k = 0; k < nx; k++) {
    d = x->data[k];
    if ((d >= leftEdge) && (d <= edges[8])) {
      bGuess = std::ceil((d - leftEdge) / delta);
      guard1 = false;
      if ((bGuess >= 1.0) && (bGuess < 9.0)) {
        low_i = static_cast<int>(bGuess) - 1;
        if ((d >= edges[low_i]) && (d < edges[static_cast<int>(bGuess)])) {
          n[low_i]++;
          bin_data[k] = static_cast<int>(bGuess);
        } else {
          guard1 = true;
        }
      } else {
        guard1 = true;
      }

      if (guard1) {
        low_i = 1;
        low_ip1 = 2;
        high_i = 9;
        while (high_i > low_ip1) {
          mid_i = (low_i + high_i) >> 1;
          if (x->data[k] >= edges[mid_i - 1]) {
            low_i = mid_i;
            low_ip1 = mid_i + 1;
          } else {
            high_i = mid_i;
          }
        }

        n[low_i - 1]++;
        bin_data[k] = low_i;
      }
    }
  }
}

void mapElementsToBins(const emxArray_real_T *x, const double edges[17], int n
  [16], int bin_data[], int bin_size[2])
{
  short unnamed_idx_1;
  int low_i;
  int nx;
  double leftEdge;
  double delta;
  int k;
  double d;
  double bGuess;
  boolean_T guard1 = false;
  int low_ip1;
  int high_i;
  int mid_i;
  std::memset(&n[0], 0, 16U * sizeof(int));
  unnamed_idx_1 = static_cast<short>(x->size[1]);
  bin_size[0] = 1;
  bin_size[1] = unnamed_idx_1;
  low_i = unnamed_idx_1;
  if (0 <= low_i - 1) {
    std::memset(&bin_data[0], 0, low_i * sizeof(int));
  }

  nx = x->size[1];
  leftEdge = edges[0];
  delta = edges[1] - edges[0];
  for (k = 0; k < nx; k++) {
    d = x->data[k];
    if ((d >= leftEdge) && (d <= edges[16])) {
      bGuess = std::ceil((d - leftEdge) / delta);
      guard1 = false;
      if ((bGuess >= 1.0) && (bGuess < 17.0)) {
        low_i = static_cast<int>(bGuess) - 1;
        if ((d >= edges[low_i]) && (d < edges[static_cast<int>(bGuess)])) {
          n[low_i]++;
          bin_data[k] = static_cast<int>(bGuess);
        } else {
          guard1 = true;
        }
      } else {
        guard1 = true;
      }

      if (guard1) {
        low_i = 1;
        low_ip1 = 2;
        high_i = 17;
        while (high_i > low_ip1) {
          mid_i = (low_i + high_i) >> 1;
          if (x->data[k] >= edges[mid_i - 1]) {
            low_i = mid_i;
            low_ip1 = mid_i + 1;
          } else {
            high_i = mid_i;
          }
        }

        n[low_i - 1]++;
        bin_data[k] = low_i;
      }
    }
  }
}

/* End of code generation (mapElementsToBins.cpp) */
