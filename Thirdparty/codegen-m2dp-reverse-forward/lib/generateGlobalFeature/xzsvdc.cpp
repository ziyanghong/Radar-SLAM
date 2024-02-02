/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * xzsvdc.cpp
 *
 * Code generation for function 'xzsvdc'
 *
 */

/* Include files */
#include "xzsvdc.h"
#include "generateGlobalFeature.h"
#include "generateGlobalFeature_emxutil.h"
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
void xzsvdc(emxArray_real_T *A, emxArray_real_T *U, double S_data[], int S_size
            [1], double V_data[], int V_size[2])
{
  int n;
  int ns;
  int minnp;
  double s_data[3];
  emxArray_real_T *work;
  double e[3];
  int i;
  double Vf[9];
  int nct;
  int nctp1;
  int k;
  int q;
  int qp1;
  int m;
  int qq;
  int nmq;
  boolean_T apply_transform;
  double nrm;
  int qjj;
  int ii;
  double r;
  double snorm;
  boolean_T exitg1;
  double scale;
  double sqds;
  double sm;
  double b;
  n = A->size[0];
  ns = A->size[0] + 1;
  if (ns >= 3) {
    ns = 3;
  }

  minnp = A->size[0];
  if (minnp >= 3) {
    minnp = 3;
  }

  ns = static_cast<short>(ns);
  if (0 <= ns - 1) {
    std::memset(&s_data[0], 0, ns * sizeof(double));
  }

  emxInit_real_T(&work, 1);
  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  i = work->size[0];
  work->size[0] = static_cast<short>(A->size[0]);
  emxEnsureCapacity_real_T(work, i);
  ns = static_cast<short>(A->size[0]);
  for (i = 0; i < ns; i++) {
    work->data[i] = 0.0;
  }

  i = U->size[0] * U->size[1];
  U->size[0] = static_cast<short>(A->size[0]);
  U->size[1] = static_cast<short>(minnp);
  emxEnsureCapacity_real_T(U, i);
  ns = static_cast<short>(A->size[0]) * static_cast<short>(minnp);
  for (i = 0; i < ns; i++) {
    U->data[i] = 0.0;
  }

  std::memset(&Vf[0], 0, 9U * sizeof(double));
  if (A->size[0] == 0) {
    Vf[0] = 1.0;
    Vf[4] = 1.0;
    Vf[8] = 1.0;
  } else {
    if (A->size[0] > 1) {
      nct = A->size[0] - 1;
    } else {
      nct = 0;
    }

    if (nct >= 3) {
      nct = 3;
    }

    nctp1 = nct + 1;
    if (nct > 1) {
      i = nct;
    } else {
      i = 1;
    }

    for (q = 0; q < i; q++) {
      qp1 = q + 2;
      qq = (q + n * q) + 1;
      nmq = n - q;
      apply_transform = false;
      if (q + 1 <= nct) {
        nrm = xnrm2(nmq, A, qq);
        if (nrm > 0.0) {
          apply_transform = true;
          if (A->data[qq - 1] < 0.0) {
            r = -nrm;
            s_data[q] = -nrm;
          } else {
            r = nrm;
            s_data[q] = nrm;
          }

          if (std::abs(r) >= 1.0020841800044864E-292) {
            nrm = 1.0 / r;
            ns = (qq + nmq) - 1;
            for (k = qq; k <= ns; k++) {
              A->data[k - 1] *= nrm;
            }
          } else {
            ns = (qq + nmq) - 1;
            for (k = qq; k <= ns; k++) {
              A->data[k - 1] /= s_data[q];
            }
          }

          A->data[qq - 1]++;
          s_data[q] = -s_data[q];
        } else {
          s_data[q] = 0.0;
        }
      }

      for (k = qp1; k < 4; k++) {
        qjj = q + n * (k - 1);
        if (apply_transform) {
          xaxpy(nmq, -(xdotc(nmq, A, qq, A, qjj + 1) / A->data[q + A->size[0] *
                       q]), qq, A, qjj + 1);
        }

        e[k - 1] = A->data[qjj];
      }

      if (q + 1 <= nct) {
        for (ii = q + 1; ii <= n; ii++) {
          U->data[(ii + U->size[0] * q) - 1] = A->data[(ii + A->size[0] * q) - 1];
        }
      }

      if (q + 1 <= 1) {
        nrm = b_xnrm2(e, 2);
        if (nrm == 0.0) {
          e[0] = 0.0;
        } else {
          if (e[1] < 0.0) {
            e[0] = -nrm;
          } else {
            e[0] = nrm;
          }

          nrm = e[0];
          if (std::abs(e[0]) >= 1.0020841800044864E-292) {
            nrm = 1.0 / e[0];
            for (k = qp1; k < 4; k++) {
              e[k - 1] *= nrm;
            }
          } else {
            for (k = qp1; k < 4; k++) {
              e[k - 1] /= nrm;
            }
          }

          e[1]++;
          e[0] = -e[0];
          if (2 <= n) {
            for (ii = qp1; ii <= n; ii++) {
              work->data[ii - 1] = 0.0;
            }

            for (k = qp1; k < 4; k++) {
              b_xaxpy(nmq - 1, e[k - 1], A, n * (k - 1) + 2, work, 2);
            }

            for (k = qp1; k < 4; k++) {
              b_xaxpy(nmq - 1, -e[k - 1] / e[1], work, 2, A, n * (k - 1) + 2);
            }
          }
        }

        for (ii = qp1; ii < 4; ii++) {
          Vf[ii - 1] = e[ii - 1];
        }
      }
    }

    if (3 < n + 1) {
      m = 2;
    } else {
      m = n;
    }

    if (nct < 3) {
      s_data[nct] = A->data[nct + A->size[0] * nct];
    }

    if (n < m + 1) {
      s_data[m] = 0.0;
    }

    if (2 < m + 1) {
      e[1] = A->data[A->size[0] * m + 1];
    }

    e[m] = 0.0;
    if (nct + 1 <= minnp) {
      for (k = nctp1; k <= minnp; k++) {
        for (ii = 0; ii < n; ii++) {
          U->data[ii + U->size[0] * (k - 1)] = 0.0;
        }

        U->data[(k + U->size[0] * (k - 1)) - 1] = 1.0;
      }
    }

    for (q = nct; q >= 1; q--) {
      qp1 = q + 1;
      ns = n - q;
      nmq = ns + 1;
      qq = (q + n * (q - 1)) - 1;
      if (s_data[q - 1] != 0.0) {
        for (k = qp1; k <= minnp; k++) {
          qjj = q + n * (k - 1);
          xaxpy(ns + 1, -(xdotc(nmq, U, qq + 1, U, qjj) / U->data[qq]), qq + 1,
                U, qjj);
        }

        for (ii = q; ii <= n; ii++) {
          U->data[(ii + U->size[0] * (q - 1)) - 1] = -U->data[(ii + U->size[0] *
            (q - 1)) - 1];
        }

        U->data[qq]++;
        for (ii = 0; ii <= q - 2; ii++) {
          U->data[ii + U->size[0] * (q - 1)] = 0.0;
        }
      } else {
        for (ii = 0; ii < n; ii++) {
          U->data[ii + U->size[0] * (q - 1)] = 0.0;
        }

        U->data[qq] = 1.0;
      }
    }

    for (q = 2; q >= 0; q--) {
      if ((q + 1 <= 1) && (e[0] != 0.0)) {
        c_xaxpy(-(b_xdotc(Vf, Vf, 5) / Vf[1]), Vf, 5);
        c_xaxpy(-(b_xdotc(Vf, Vf, 8) / Vf[1]), Vf, 8);
      }

      Vf[3 * q] = 0.0;
      Vf[3 * q + 1] = 0.0;
      Vf[3 * q + 2] = 0.0;
      Vf[q + 3 * q] = 1.0;
    }

    nctp1 = m;
    qjj = 0;
    snorm = 0.0;
    for (q = 0; q <= m; q++) {
      if (s_data[q] != 0.0) {
        nrm = std::abs(s_data[q]);
        r = s_data[q] / nrm;
        s_data[q] = nrm;
        if (q + 1 < m + 1) {
          e[q] /= r;
        }

        if (q + 1 <= n) {
          ns = n * q;
          i = ns + n;
          for (k = ns + 1; k <= i; k++) {
            U->data[k - 1] *= r;
          }
        }
      }

      if ((q + 1 < m + 1) && (e[q] != 0.0)) {
        nrm = std::abs(e[q]);
        r = nrm / e[q];
        e[q] = nrm;
        s_data[q + 1] *= r;
        ns = 3 * (q + 1);
        i = ns + 3;
        for (k = ns + 1; k <= i; k++) {
          Vf[k - 1] *= r;
        }
      }

      nrm = std::abs(s_data[q]);
      r = std::abs(e[q]);
      if ((nrm > r) || rtIsNaN(r)) {
        r = nrm;
      }

      if ((!(snorm > r)) && (!rtIsNaN(r))) {
        snorm = r;
      }
    }

    while ((m + 1 > 0) && (qjj < 75)) {
      ii = m;
      exitg1 = false;
      while (!(exitg1 || (ii == 0))) {
        nrm = std::abs(e[ii - 1]);
        if ((nrm <= 2.2204460492503131E-16 * (std::abs(s_data[ii - 1]) + std::
              abs(s_data[ii]))) || (nrm <= 1.0020841800044864E-292) || ((qjj >
              20) && (nrm <= 2.2204460492503131E-16 * snorm))) {
          e[ii - 1] = 0.0;
          exitg1 = true;
        } else {
          ii--;
        }
      }

      if (ii == m) {
        ns = 4;
      } else {
        nmq = m + 1;
        ns = m + 1;
        exitg1 = false;
        while ((!exitg1) && (ns >= ii)) {
          nmq = ns;
          if (ns == ii) {
            exitg1 = true;
          } else {
            nrm = 0.0;
            if (ns < m + 1) {
              nrm = std::abs(e[ns - 1]);
            }

            if (ns > ii + 1) {
              nrm += std::abs(e[ns - 2]);
            }

            r = std::abs(s_data[ns - 1]);
            if ((r <= 2.2204460492503131E-16 * nrm) || (r <=
                 1.0020841800044864E-292)) {
              s_data[ns - 1] = 0.0;
              exitg1 = true;
            } else {
              ns--;
            }
          }
        }

        if (nmq == ii) {
          ns = 3;
        } else if (nmq == m + 1) {
          ns = 1;
        } else {
          ns = 2;
          ii = nmq;
        }
      }

      switch (ns) {
       case 1:
        r = e[m - 1];
        e[m - 1] = 0.0;
        for (k = m; k >= ii + 1; k--) {
          xrotg(&s_data[k - 1], &r, &sqds, &sm);
          if (k > ii + 1) {
            r = -sm * e[0];
            e[0] *= sqds;
          }

          xrot(Vf, 3 * (k - 1) + 1, 3 * m + 1, sqds, sm);
        }
        break;

       case 2:
        r = e[ii - 1];
        e[ii - 1] = 0.0;
        for (k = ii + 1; k <= m + 1; k++) {
          xrotg(&s_data[k - 1], &r, &sqds, &sm);
          b = e[k - 1];
          r = -sm * b;
          e[k - 1] = b * sqds;
          b_xrot(n, U, n * (k - 1) + 1, n * (ii - 1) + 1, sqds, sm);
        }
        break;

       case 3:
        scale = std::abs(s_data[m]);
        nrm = s_data[m - 1];
        r = std::abs(nrm);
        if ((!(scale > r)) && (!rtIsNaN(r))) {
          scale = r;
        }

        b = e[m - 1];
        r = std::abs(b);
        if ((!(scale > r)) && (!rtIsNaN(r))) {
          scale = r;
        }

        r = std::abs(s_data[ii]);
        if ((!(scale > r)) && (!rtIsNaN(r))) {
          scale = r;
        }

        r = std::abs(e[ii]);
        if ((!(scale > r)) && (!rtIsNaN(r))) {
          scale = r;
        }

        sm = s_data[m] / scale;
        nrm /= scale;
        r = b / scale;
        sqds = s_data[ii] / scale;
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
        nrm = sqds * (e[ii] / scale);
        for (k = ii + 1; k <= m; k++) {
          xrotg(&r, &nrm, &sqds, &sm);
          if (k > ii + 1) {
            e[0] = r;
          }

          nrm = e[k - 1];
          b = s_data[k - 1];
          e[k - 1] = sqds * nrm - sm * b;
          r = sm * s_data[k];
          s_data[k] *= sqds;
          xrot(Vf, 3 * (k - 1) + 1, 3 * k + 1, sqds, sm);
          s_data[k - 1] = sqds * b + sm * nrm;
          xrotg(&s_data[k - 1], &r, &sqds, &sm);
          r = sqds * e[k - 1] + sm * s_data[k];
          s_data[k] = -sm * e[k - 1] + sqds * s_data[k];
          nrm = sm * e[k];
          e[k] *= sqds;
          if (k < n) {
            b_xrot(n, U, n * (k - 1) + 1, n * k + 1, sqds, sm);
          }
        }

        e[m - 1] = r;
        qjj++;
        break;

       default:
        if (s_data[ii] < 0.0) {
          s_data[ii] = -s_data[ii];
          ns = 3 * ii;
          i = ns + 3;
          for (k = ns + 1; k <= i; k++) {
            Vf[k - 1] = -Vf[k - 1];
          }
        }

        qp1 = ii + 1;
        while ((ii + 1 < nctp1 + 1) && (s_data[ii] < s_data[qp1])) {
          nrm = s_data[ii];
          s_data[ii] = s_data[qp1];
          s_data[qp1] = nrm;
          xswap(Vf, 3 * ii + 1, 3 * (ii + 1) + 1);
          if (ii + 1 < n) {
            b_xswap(n, U, n * ii + 1, n * (ii + 1) + 1);
          }

          ii = qp1;
          qp1++;
        }

        qjj = 0;
        m--;
        break;
      }
    }
  }

  emxFree_real_T(&work);
  S_size[0] = minnp;
  V_size[0] = 3;
  V_size[1] = minnp;
  for (k = 0; k < minnp; k++) {
    S_data[k] = s_data[k];
    V_data[3 * k] = Vf[3 * k];
    i = 3 * k + 1;
    V_data[i] = Vf[i];
    i = 3 * k + 2;
    V_data[i] = Vf[i];
  }
}

/* End of code generation (xzsvdc.cpp) */
