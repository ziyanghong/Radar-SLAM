/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * svd1.cpp
 *
 * Code generation for function 'svd1'
 *
 */

/* Include files */
#include "svd1.h"
#include "generateGlobalFeature.h"
#include "rt_nonfinite.h"
#include "xaxpy.h"
#include "xdotc.h"
#include "xnrm2.h"
#include "xrot.h"
#include "xrotg.h"
#include "xswap.h"
#include <cmath>
#include <cstring>

/* Function Definitions */
void b_svd(const double A[8192], double U[4096], double s[64], double V[16384])
{
  double b_A[8192];
  double b_s[65];
  double e[128];
  double work[64];
  int q;
  int m;
  int qp1;
  int iter;
  int qq;
  boolean_T apply_transform;
  double nrm;
  int k;
  int qjj;
  double snorm;
  int qp1jj;
  double r;
  int exitg1;
  boolean_T exitg2;
  double scale;
  double sm;
  double sqds;
  double b;
  std::memcpy(&b_A[0], &A[0], 8192U * sizeof(double));
  std::memset(&b_s[0], 0, 65U * sizeof(double));
  std::memset(&e[0], 0, 128U * sizeof(double));
  std::memset(&work[0], 0, 64U * sizeof(double));
  std::memset(&U[0], 0, 4096U * sizeof(double));
  std::memset(&V[0], 0, 16384U * sizeof(double));
  for (q = 0; q < 64; q++) {
    qp1 = q + 2;
    iter = q << 6;
    qq = (q + iter) + 1;
    apply_transform = false;
    if (q + 1 <= 63) {
      nrm = c_xnrm2(64 - q, b_A, qq);
      if (nrm > 0.0) {
        apply_transform = true;
        if (b_A[qq - 1] < 0.0) {
          nrm = -nrm;
        }

        b_s[q] = nrm;
        if (std::abs(nrm) >= 1.0020841800044864E-292) {
          nrm = 1.0 / nrm;
          qp1jj = (qq - q) + 63;
          for (k = qq; k <= qp1jj; k++) {
            b_A[k - 1] *= nrm;
          }
        } else {
          qp1jj = (qq - q) + 63;
          for (k = qq; k <= qp1jj; k++) {
            b_A[k - 1] /= b_s[q];
          }
        }

        b_A[qq - 1]++;
        b_s[q] = -b_s[q];
      } else {
        b_s[q] = 0.0;
      }
    }

    for (k = qp1; k < 129; k++) {
      qjj = q + ((k - 1) << 6);
      if (apply_transform) {
        d_xaxpy(64 - q, -(c_xdotc(64 - q, b_A, qq, b_A, qjj + 1) / b_A[q + (q <<
                  6)]), qq, b_A, qjj + 1);
      }

      e[k - 1] = b_A[qjj];
    }

    if (q + 1 <= 63) {
      for (k = q + 1; k < 65; k++) {
        qp1jj = (k + iter) - 1;
        U[qp1jj] = b_A[qp1jj];
      }
    }

    nrm = d_xnrm2(127 - q, e, q + 2);
    if (nrm == 0.0) {
      e[q] = 0.0;
    } else {
      if (e[q + 1] < 0.0) {
        e[q] = -nrm;
      } else {
        e[q] = nrm;
      }

      nrm = e[q];
      if (std::abs(e[q]) >= 1.0020841800044864E-292) {
        nrm = 1.0 / e[q];
        for (k = qp1; k < 129; k++) {
          e[k - 1] *= nrm;
        }
      } else {
        for (k = qp1; k < 129; k++) {
          e[k - 1] /= nrm;
        }
      }

      e[q + 1]++;
      e[q] = -e[q];
      if (q + 2 <= 64) {
        for (k = qp1; k < 65; k++) {
          work[k - 1] = 0.0;
        }

        for (k = qp1; k < 129; k++) {
          e_xaxpy(63 - q, e[k - 1], b_A, (q + ((k - 1) << 6)) + 2, work, q + 2);
        }

        for (k = qp1; k < 129; k++) {
          f_xaxpy(63 - q, -e[k - 1] / e[q + 1], work, q + 2, b_A, (q + ((k - 1) <<
                    6)) + 2);
        }
      }
    }

    for (k = qp1; k < 129; k++) {
      V[(k + (q << 7)) - 1] = e[k - 1];
    }
  }

  m = 63;
  b_s[63] = b_A[4095];
  b_s[64] = 0.0;
  e[64] = 0.0;
  std::memset(&U[4032], 0, 64U * sizeof(double));
  U[4095] = 1.0;
  for (q = 62; q >= 0; q--) {
    qp1 = q + 2;
    iter = q << 6;
    qq = q + iter;
    if (b_s[q] != 0.0) {
      for (k = qp1; k < 65; k++) {
        qjj = (q + ((k - 1) << 6)) + 1;
        g_xaxpy(64 - q, -(d_xdotc(64 - q, U, qq + 1, U, qjj) / U[qq]), qq + 1, U,
                qjj);
      }

      for (k = q + 1; k < 65; k++) {
        qp1jj = (k + iter) - 1;
        U[qp1jj] = -U[qp1jj];
      }

      U[qq]++;
      for (k = 0; k < q; k++) {
        U[k + iter] = 0.0;
      }
    } else {
      std::memset(&U[iter], 0, 64U * sizeof(double));
      U[qq] = 1.0;
    }
  }

  for (q = 127; q >= 0; q--) {
    if ((q + 1 <= 64) && (e[q] != 0.0)) {
      qp1 = q + 2;
      qjj = (q + (q << 7)) + 2;
      for (k = qp1; k < 129; k++) {
        qp1jj = (q + ((k - 1) << 7)) + 2;
        h_xaxpy(127 - q, -(e_xdotc(127 - q, V, qjj, V, qp1jj) / V[qjj - 1]), qjj,
                V, qp1jj);
      }
    }

    std::memset(&V[q * 128], 0, 128U * sizeof(double));
    V[q + (q << 7)] = 1.0;
  }

  iter = 0;
  snorm = 0.0;
  for (q = 0; q < 65; q++) {
    if (b_s[q] != 0.0) {
      nrm = std::abs(b_s[q]);
      r = b_s[q] / nrm;
      b_s[q] = nrm;
      if (q + 1 < 65) {
        e[q] /= r;
      }

      if (q + 1 <= 64) {
        qjj = q << 6;
        qp1jj = qjj + 64;
        for (k = qjj + 1; k <= qp1jj; k++) {
          U[k - 1] *= r;
        }
      }
    }

    if ((q + 1 < 65) && (e[q] != 0.0)) {
      nrm = std::abs(e[q]);
      r = nrm / e[q];
      e[q] = nrm;
      b_s[q + 1] *= r;
      qjj = (q + 1) << 7;
      qp1jj = qjj + 128;
      for (k = qjj + 1; k <= qp1jj; k++) {
        V[k - 1] *= r;
      }
    }

    nrm = std::abs(b_s[q]);
    r = std::abs(e[q]);
    if ((nrm > r) || rtIsNaN(r)) {
      r = nrm;
    }

    if ((!(snorm > r)) && (!rtIsNaN(r))) {
      snorm = r;
    }
  }

  while ((m + 2 > 0) && (iter < 75)) {
    k = m;
    do {
      exitg1 = 0;
      q = k + 1;
      if (k + 1 == 0) {
        exitg1 = 1;
      } else {
        nrm = std::abs(e[k]);
        if ((nrm <= 2.2204460492503131E-16 * (std::abs(b_s[k]) + std::abs(b_s[k
               + 1]))) || (nrm <= 1.0020841800044864E-292) || ((iter > 20) &&
             (nrm <= 2.2204460492503131E-16 * snorm))) {
          e[k] = 0.0;
          exitg1 = 1;
        } else {
          k--;
        }
      }
    } while (exitg1 == 0);

    if (k + 1 == m + 1) {
      qjj = 4;
    } else {
      qp1jj = m + 2;
      qjj = m + 2;
      exitg2 = false;
      while ((!exitg2) && (qjj >= k + 1)) {
        qp1jj = qjj;
        if (qjj == k + 1) {
          exitg2 = true;
        } else {
          nrm = 0.0;
          if (qjj < m + 2) {
            nrm = std::abs(e[qjj - 1]);
          }

          if (qjj > k + 2) {
            nrm += std::abs(e[qjj - 2]);
          }

          r = std::abs(b_s[qjj - 1]);
          if ((r <= 2.2204460492503131E-16 * nrm) || (r <=
               1.0020841800044864E-292)) {
            b_s[qjj - 1] = 0.0;
            exitg2 = true;
          } else {
            qjj--;
          }
        }
      }

      if (qp1jj == k + 1) {
        qjj = 3;
      } else if (qp1jj == m + 2) {
        qjj = 1;
      } else {
        qjj = 2;
        q = qp1jj;
      }
    }

    switch (qjj) {
     case 1:
      r = e[m];
      e[m] = 0.0;
      qp1jj = m + 1;
      for (k = qp1jj; k >= q + 1; k--) {
        xrotg(&b_s[k - 1], &r, &sm, &sqds);
        if (k > q + 1) {
          b = e[k - 2];
          r = -sqds * b;
          e[k - 2] = b * sm;
        }

        c_xrot(V, ((k - 1) << 7) + 1, ((m + 1) << 7) + 1, sm, sqds);
      }
      break;

     case 2:
      r = e[q - 1];
      e[q - 1] = 0.0;
      for (k = q + 1; k <= m + 2; k++) {
        xrotg(&b_s[k - 1], &r, &sm, &sqds);
        b = e[k - 1];
        r = -sqds * b;
        e[k - 1] = b * sm;
        d_xrot(U, ((k - 1) << 6) + 1, ((q - 1) << 6) + 1, sm, sqds);
      }
      break;

     case 3:
      qjj = m + 1;
      nrm = b_s[m + 1];
      scale = std::abs(nrm);
      r = std::abs(b_s[m]);
      if ((!(scale > r)) && (!rtIsNaN(r))) {
        scale = r;
      }

      r = std::abs(e[m]);
      if ((!(scale > r)) && (!rtIsNaN(r))) {
        scale = r;
      }

      r = std::abs(b_s[q]);
      if ((!(scale > r)) && (!rtIsNaN(r))) {
        scale = r;
      }

      r = std::abs(e[q]);
      if ((!(scale > r)) && (!rtIsNaN(r))) {
        scale = r;
      }

      sm = nrm / scale;
      nrm = b_s[m] / scale;
      r = e[m] / scale;
      sqds = b_s[q] / scale;
      b = ((nrm + sm) * (nrm - sm) + r * r) / 2.0;
      nrm = sm * r;
      nrm *= nrm;
      if ((b != 0.0) || (nrm != 0.0)) {
        r = std::sqrt(b * b + nrm);
        if (b < 0.0) {
          r = -r;
        }

        r = nrm / (b + r);
      } else {
        r = 0.0;
      }

      r += (sqds + sm) * (sqds - sm);
      nrm = sqds * (e[q] / scale);
      for (k = q + 1; k <= qjj; k++) {
        xrotg(&r, &nrm, &sm, &sqds);
        if (k > q + 1) {
          e[k - 2] = r;
        }

        nrm = e[k - 1];
        b = b_s[k - 1];
        e[k - 1] = sm * nrm - sqds * b;
        r = sqds * b_s[k];
        b_s[k] *= sm;
        c_xrot(V, ((k - 1) << 7) + 1, (k << 7) + 1, sm, sqds);
        b_s[k - 1] = sm * b + sqds * nrm;
        xrotg(&b_s[k - 1], &r, &sm, &sqds);
        r = sm * e[k - 1] + sqds * b_s[k];
        b_s[k] = -sqds * e[k - 1] + sm * b_s[k];
        nrm = sqds * e[k];
        e[k] *= sm;
        if (k < 64) {
          d_xrot(U, ((k - 1) << 6) + 1, (k << 6) + 1, sm, sqds);
        }
      }

      e[m] = r;
      iter++;
      break;

     default:
      if (b_s[q] < 0.0) {
        b_s[q] = -b_s[q];
        qjj = q << 7;
        qp1jj = qjj + 128;
        for (k = qjj + 1; k <= qp1jj; k++) {
          V[k - 1] = -V[k - 1];
        }
      }

      qp1 = q + 1;
      while ((q + 1 < 65) && (b_s[q] < b_s[qp1])) {
        nrm = b_s[q];
        b_s[q] = b_s[qp1];
        b_s[qp1] = nrm;
        c_xswap(V, (q << 7) + 1, ((q + 1) << 7) + 1);
        if (q + 1 < 64) {
          d_xswap(U, (q << 6) + 1, ((q + 1) << 6) + 1);
        }

        q = qp1;
        qp1++;
      }

      iter = 0;
      m--;
      break;
    }
  }

  std::memcpy(&s[0], &b_s[0], 64U * sizeof(double));
}

/* End of code generation (svd1.cpp) */
