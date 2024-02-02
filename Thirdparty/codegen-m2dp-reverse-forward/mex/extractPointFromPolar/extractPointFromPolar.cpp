/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * extractPointFromPolar.cpp
 *
 * Code generation for function 'extractPointFromPolar'
 *
 */

/* Include files */
#include "extractPointFromPolar.h"
#include "eml_int_forloop_overflow_check.h"
#include "extractPointFromPolar_data.h"
#include "extractPointFromPolar_emxutil.h"
#include "findpeaks.h"
#include "mwmathutil.h"
#include "rt_nonfinite.h"
#include "std.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo emlrtRSI = { 40,    /* lineNo */
  "extractPointFromPolar",             /* fcnName */
  "/home/hong/catkin_ws/src/radar_localization/Thirdparty/extractPointFromPolar.m"/* pathName */
};

static emlrtRSInfo b_emlrtRSI = { 39,  /* lineNo */
  "extractPointFromPolar",             /* fcnName */
  "/home/hong/catkin_ws/src/radar_localization/Thirdparty/extractPointFromPolar.m"/* pathName */
};

static emlrtRSInfo c_emlrtRSI = { 37,  /* lineNo */
  "extractPointFromPolar",             /* fcnName */
  "/home/hong/catkin_ws/src/radar_localization/Thirdparty/extractPointFromPolar.m"/* pathName */
};

static emlrtRSInfo ce_emlrtRSI = { 49, /* lineNo */
  "mean",                              /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/datafun/mean.m"/* pathName */
};

static emlrtBCInfo emlrtBCI = { 1,     /* iFirst */
  10000,                               /* iLast */
  61,                                  /* lineNo */
  41,                                  /* colNo */
  "pointCloudForward",                 /* aName */
  "extractPointFromPolar",             /* fName */
  "/home/hong/catkin_ws/src/radar_localization/Thirdparty/extractPointFromPolar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo b_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  48,                                  /* lineNo */
  22,                                  /* colNo */
  "locs",                              /* aName */
  "extractPointFromPolar",             /* fName */
  "/home/hong/catkin_ws/src/radar_localization/Thirdparty/extractPointFromPolar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo c_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  47,                                  /* lineNo */
  22,                                  /* colNo */
  "locs",                              /* aName */
  "extractPointFromPolar",             /* fName */
  "/home/hong/catkin_ws/src/radar_localization/Thirdparty/extractPointFromPolar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo d_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  44,                                  /* lineNo */
  18,                                  /* colNo */
  "pks",                               /* aName */
  "extractPointFromPolar",             /* fName */
  "/home/hong/catkin_ws/src/radar_localization/Thirdparty/extractPointFromPolar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo e_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  35,                                  /* lineNo */
  23,                                  /* colNo */
  "img",                               /* aName */
  "extractPointFromPolar",             /* fName */
  "/home/hong/catkin_ws/src/radar_localization/Thirdparty/extractPointFromPolar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo f_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  23,                                  /* lineNo */
  17,                                  /* colNo */
  "img",                               /* aName */
  "extractPointFromPolar",             /* fName */
  "/home/hong/catkin_ws/src/radar_localization/Thirdparty/extractPointFromPolar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo g_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  23,                                  /* lineNo */
  7,                                   /* colNo */
  "img",                               /* aName */
  "extractPointFromPolar",             /* fName */
  "/home/hong/catkin_ws/src/radar_localization/Thirdparty/extractPointFromPolar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo h_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  22,                                  /* lineNo */
  9,                                   /* colNo */
  "img",                               /* aName */
  "extractPointFromPolar",             /* fName */
  "/home/hong/catkin_ws/src/radar_localization/Thirdparty/extractPointFromPolar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo i_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  22,                                  /* lineNo */
  7,                                   /* colNo */
  "img",                               /* aName */
  "extractPointFromPolar",             /* fName */
  "/home/hong/catkin_ws/src/radar_localization/Thirdparty/extractPointFromPolar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo j_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  14,                                  /* lineNo */
  15,                                  /* colNo */
  "img",                               /* aName */
  "extractPointFromPolar",             /* fName */
  "/home/hong/catkin_ws/src/radar_localization/Thirdparty/extractPointFromPolar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo emlrtDCI = { 14,    /* lineNo */
  15,                                  /* colNo */
  "extractPointFromPolar",             /* fName */
  "/home/hong/catkin_ws/src/radar_localization/Thirdparty/extractPointFromPolar.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo k_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  14,                                  /* lineNo */
  13,                                  /* colNo */
  "img",                               /* aName */
  "extractPointFromPolar",             /* fName */
  "/home/hong/catkin_ws/src/radar_localization/Thirdparty/extractPointFromPolar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo l_emlrtBCI = { 1,   /* iFirst */
  10000,                               /* iLast */
  52,                                  /* lineNo */
  31,                                  /* colNo */
  "pointCloudForward",                 /* aName */
  "extractPointFromPolar",             /* fName */
  "/home/hong/catkin_ws/src/radar_localization/Thirdparty/extractPointFromPolar.m",/* pName */
  3                                    /* checkKind */
};

static emlrtRTEInfo r_emlrtRTEI = { 14,/* lineNo */
  7,                                   /* colNo */
  "extractPointFromPolar",             /* fName */
  "/home/hong/catkin_ws/src/radar_localization/Thirdparty/extractPointFromPolar.m"/* pName */
};

static emlrtRTEInfo s_emlrtRTEI = { 14,/* lineNo */
  1,                                   /* colNo */
  "extractPointFromPolar",             /* fName */
  "/home/hong/catkin_ws/src/radar_localization/Thirdparty/extractPointFromPolar.m"/* pName */
};

static emlrtRTEInfo t_emlrtRTEI = { 15,/* lineNo */
  1,                                   /* colNo */
  "extractPointFromPolar",             /* fName */
  "/home/hong/catkin_ws/src/radar_localization/Thirdparty/extractPointFromPolar.m"/* pName */
};

static emlrtRTEInfo u_emlrtRTEI = { 61,/* lineNo */
  1,                                   /* colNo */
  "extractPointFromPolar",             /* fName */
  "/home/hong/catkin_ws/src/radar_localization/Thirdparty/extractPointFromPolar.m"/* pName */
};

static emlrtRTEInfo v_emlrtRTEI = { 35,/* lineNo */
  12,                                  /* colNo */
  "extractPointFromPolar",             /* fName */
  "/home/hong/catkin_ws/src/radar_localization/Thirdparty/extractPointFromPolar.m"/* pName */
};

static emlrtRTEInfo w_emlrtRTEI = { 37,/* lineNo */
  6,                                   /* colNo */
  "extractPointFromPolar",             /* fName */
  "/home/hong/catkin_ws/src/radar_localization/Thirdparty/extractPointFromPolar.m"/* pName */
};

static emlrtRTEInfo x_emlrtRTEI = { 1, /* lineNo */
  31,                                  /* colNo */
  "extractPointFromPolar",             /* fName */
  "/home/hong/catkin_ws/src/radar_localization/Thirdparty/extractPointFromPolar.m"/* pName */
};

/* Function Definitions */
void extractPointFromPolar(extractPointFromPolarStackData *SD, const emlrtStack *
  sp, emxArray_uint8_T *img, real_T max_selected_distance, real_T
  radar_resolution, real_T max_distance, real_T num_points_per_beam,
  emxArray_real_T *pointCloudForward)
{
  real_T max_pixel;
  int32_T loop_ub;
  emxArray_uint8_T *b_img;
  int32_T car_reflection;
  int32_T i;
  int32_T i1;
  emxArray_real_T *t;
  int32_T k;
  emxArray_real_T *c_img;
  int32_T counter;
  emxArray_real_T *pks;
  emxArray_real_T *locs;
  int16_T i2;
  static const int16_T iv[342] = { 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
    21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39,
    40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58,
    59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77,
    78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96,
    97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112,
    113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127,
    128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142,
    143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157,
    158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172,
    173, 174, 175, 176, 177, 178, 179, 180, 220, 221, 222, 223, 224, 225, 226,
    227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241,
    242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 256,
    257, 258, 259, 260, 261, 262, 263, 264, 265, 266, 267, 268, 269, 270, 271,
    272, 273, 274, 275, 276, 277, 278, 279, 280, 281, 282, 283, 284, 285, 286,
    287, 288, 289, 290, 291, 292, 293, 294, 295, 296, 297, 298, 299, 300, 301,
    302, 303, 304, 305, 306, 307, 308, 309, 310, 311, 312, 313, 314, 315, 316,
    317, 318, 319, 320, 321, 322, 323, 324, 325, 326, 327, 328, 329, 330, 331,
    332, 333, 334, 335, 336, 337, 338, 339, 340, 341, 342, 343, 344, 345, 346,
    347, 348, 349, 350, 351, 352, 353, 354, 355, 356, 357, 358, 359, 360, 361,
    362, 363, 364, 365, 366, 367, 368, 369, 370, 371, 372, 373, 374, 375, 376,
    377, 378, 379, 380, 381, 382, 383, 384, 385, 386, 387, 388, 389, 390 };

  real_T peaks_std;
  real_T num_points_per_beam_counter;
  boolean_T exitg1;
  boolean_T guard1 = false;
  real_T theta;
  real_T pointCloudForward_tmp;
  emlrtStack st;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);

  /*  input: */
  /*  img: polar image */
  /*  max_selected_distance: maximum distance chosen by user in meters */
  /*  radar_resolution: m / pixel */
  /*  max_distance: maximum distance for each azimuth sacn in meters */
  max_pixel = muDoubleScalarFloor(max_selected_distance / max_distance *
    static_cast<real_T>(img->size[1]));
  if (1.0 > max_pixel) {
    loop_ub = 0;
  } else {
    if (1 > img->size[1]) {
      emlrtDynamicBoundsCheckR2012b(1, 1, img->size[1], &k_emlrtBCI, sp);
    }

    loop_ub = static_cast<int32_T>(max_pixel);
    if (max_pixel != loop_ub) {
      emlrtIntegerCheckR2012b(max_pixel, &emlrtDCI, sp);
    }

    if ((loop_ub < 1) || (loop_ub > img->size[1])) {
      emlrtDynamicBoundsCheckR2012b(loop_ub, 1, img->size[1], &j_emlrtBCI, sp);
    }
  }

  emxInit_uint8_T(sp, &b_img, 2, &r_emlrtRTEI, true);
  car_reflection = img->size[0] - 1;
  i = b_img->size[0] * b_img->size[1];
  b_img->size[0] = img->size[0];
  b_img->size[1] = loop_ub;
  emxEnsureCapacity_uint8_T(sp, b_img, i, &r_emlrtRTEI);
  for (i = 0; i < loop_ub; i++) {
    for (i1 = 0; i1 <= car_reflection; i1++) {
      b_img->data[i1 + b_img->size[0] * i] = img->data[i1 + img->size[0] * i];
    }
  }

  i = img->size[0] * img->size[1];
  img->size[0] = b_img->size[0];
  img->size[1] = b_img->size[1];
  emxEnsureCapacity_uint8_T(sp, img, i, &s_emlrtRTEI);
  loop_ub = b_img->size[0] * b_img->size[1];
  for (i = 0; i < loop_ub; i++) {
    img->data[i] = b_img->data[i];
  }

  emxFree_uint8_T(&b_img);
  emxInit_real_T(sp, &t, 2, &t_emlrtRTEI, true);
  if (img->size[1] < 1) {
    t->size[0] = 1;
    t->size[1] = 0;
  } else {
    i = t->size[0] * t->size[1];
    t->size[0] = 1;
    t->size[1] = static_cast<int32_T>((static_cast<real_T>(img->size[1]) - 1.0))
      + 1;
    emxEnsureCapacity_real_T(sp, t, i, &t_emlrtRTEI);
    loop_ub = static_cast<int32_T>((static_cast<real_T>(img->size[1]) - 1.0));
    for (i = 0; i <= loop_ub; i++) {
      t->data[i] = static_cast<real_T>(i) + 1.0;
    }
  }

  if (max_pixel > 600.0) {
    car_reflection = 71;
  } else {
    car_reflection = 30;
  }

  if (1 > img->size[1]) {
    emlrtDynamicBoundsCheckR2012b(1, 1, img->size[1], &i_emlrtBCI, sp);
  }

  if (car_reflection > img->size[1]) {
    emlrtDynamicBoundsCheckR2012b(car_reflection, 1, img->size[1], &h_emlrtBCI,
      sp);
  }

  loop_ub = img->size[0];
  for (i = 0; i < car_reflection; i++) {
    for (i1 = 0; i1 < loop_ub; i1++) {
      img->data[i1 + img->size[0] * i] = 0U;
    }
  }

  if (max_pixel > img->size[1]) {
    i = 0;
    i1 = 0;
  } else {
    i = static_cast<int32_T>(max_pixel);
    if ((i < 1) || (i > img->size[1])) {
      emlrtDynamicBoundsCheckR2012b(i, 1, img->size[1], &g_emlrtBCI, sp);
    }

    i--;
    if (img->size[1] < 1) {
      emlrtDynamicBoundsCheckR2012b(img->size[1], 1, img->size[1], &f_emlrtBCI,
        sp);
    }

    i1 = img->size[1];
  }

  loop_ub = img->size[0];
  car_reflection = i1 - i;
  for (i1 = 0; i1 < car_reflection; i1++) {
    for (k = 0; k < loop_ub; k++) {
      img->data[k + img->size[0] * (i + i1)] = 0U;
    }
  }

  /*  allocated_memoery = 2000; % defalut */
  memset(&SD->f0.pointCloudForward[0], 0, 30000U * sizeof(real_T));
  emxInit_real_T(sp, &c_img, 2, &v_emlrtRTEI, true);

  /*  tic */
  counter = 1;

  /*  for i=1:size(img,1) */
  loop_ub = img->size[1];
  emxInit_real_T(sp, &pks, 2, &w_emlrtRTEI, true);
  emxInit_real_T(sp, &locs, 2, &x_emlrtRTEI, true);
  for (k = 0; k < 342; k++) {
    i2 = iv[k];
    if (iv[k] > img->size[0]) {
      emlrtDynamicBoundsCheckR2012b(static_cast<int32_T>(iv[k]), 1, img->size[0],
        &e_emlrtBCI, sp);
    }

    /*      tic */
    i = c_img->size[0] * c_img->size[1];
    c_img->size[0] = 1;
    c_img->size[1] = loop_ub;
    emxEnsureCapacity_real_T(sp, c_img, i, &v_emlrtRTEI);
    for (i = 0; i < loop_ub; i++) {
      c_img->data[i] = img->data[(i2 + img->size[0] * i) - 1];
    }

    st.site = &c_emlrtRSI;
    findpeaks(&st, c_img, t, pks, locs);

    /*      toc */
    st.site = &b_emlrtRSI;
    i = pks->size[1];
    b_st.site = &ce_emlrtRSI;
    if (pks->size[1] == 0) {
      max_pixel = 0.0;
    } else {
      c_st.site = &de_emlrtRSI;
      max_pixel = pks->data[0];
      d_st.site = &ee_emlrtRSI;
      if ((2 <= pks->size[1]) && (pks->size[1] > 2147483646)) {
        e_st.site = &u_emlrtRSI;
        check_forloop_overflow_error(&e_st);
      }

      for (car_reflection = 2; car_reflection <= i; car_reflection++) {
        max_pixel += pks->data[car_reflection - 1];
      }
    }

    max_pixel /= static_cast<real_T>(pks->size[1]);
    st.site = &emlrtRSI;
    peaks_std = b_std(&st, pks);
    num_points_per_beam_counter = 0.0;
    car_reflection = 0;
    exitg1 = false;
    while ((!exitg1) && (car_reflection <= pks->size[1] - 1)) {
      i = car_reflection + 1;
      if ((i < 1) || (i > pks->size[1])) {
        emlrtDynamicBoundsCheckR2012b(i, 1, pks->size[1], &d_emlrtBCI, sp);
      }

      guard1 = false;
      if (pks->data[car_reflection] > max_pixel + peaks_std) {
        theta = -static_cast<real_T>(i2) * 2.0 * 3.1415926535897931 / 400.0;
        i = car_reflection + 1;
        if ((i < 1) || (i > locs->size[1])) {
          emlrtDynamicBoundsCheckR2012b(i, 1, locs->size[1], &c_emlrtBCI, sp);
        }

        i = car_reflection + 1;
        if ((i < 1) || (i > locs->size[1])) {
          emlrtDynamicBoundsCheckR2012b(i, 1, locs->size[1], &b_emlrtBCI, sp);
        }

        /*  Forward view */
        if (counter > 10000) {
          emlrtDynamicBoundsCheckR2012b(10001, 1, 10000, &l_emlrtBCI, sp);
        }

        pointCloudForward_tmp = locs->data[car_reflection] * radar_resolution;
        SD->f0.pointCloudForward[counter - 1] = pointCloudForward_tmp *
          muDoubleScalarCos(theta);
        SD->f0.pointCloudForward[counter + 9999] = pointCloudForward_tmp *
          muDoubleScalarSin(theta);
        SD->f0.pointCloudForward[counter + 19999] = 1.0;
        num_points_per_beam_counter++;
        counter++;
        if (num_points_per_beam_counter >= num_points_per_beam) {
          exitg1 = true;
        } else {
          guard1 = true;
        }
      } else {
        guard1 = true;
      }

      if (guard1) {
        car_reflection++;
        if (*emlrtBreakCheckR2012bFlagVar != 0) {
          emlrtBreakCheckR2012b(sp);
        }
      }
    }

    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(sp);
    }
  }

  emxFree_real_T(&c_img);
  emxFree_real_T(&locs);
  emxFree_real_T(&pks);
  emxFree_real_T(&t);
  if (counter > 10000) {
    emlrtDynamicBoundsCheckR2012b(10001, 1, 10000, &emlrtBCI, sp);
  }

  i = pointCloudForward->size[0] * pointCloudForward->size[1];
  pointCloudForward->size[0] = counter;
  pointCloudForward->size[1] = 3;
  emxEnsureCapacity_real_T(sp, pointCloudForward, i, &u_emlrtRTEI);
  for (i = 0; i < 3; i++) {
    for (i1 = 0; i1 < counter; i1++) {
      pointCloudForward->data[i1 + pointCloudForward->size[0] * i] =
        SD->f0.pointCloudForward[i1 + 10000 * i];
    }
  }

  /*  toc */
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (extractPointFromPolar.cpp) */
