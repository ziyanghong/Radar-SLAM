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
#include "eml_int_forloop_overflow_check.h"
#include "eml_setop.h"
#include "extractPointFromPolar.h"
#include "extractPointFromPolar_data.h"
#include "extractPointFromPolar_emxutil.h"
#include "flipud.h"
#include "indexShapeCheck.h"
#include "mwmathutil.h"
#include "rt_nonfinite.h"
#include "scalexpAlloc.h"
#include "sort.h"

/* Variable Definitions */
static emlrtRSInfo d_emlrtRSI = { 135, /* lineNo */
  "findpeaks",                         /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pathName */
};

static emlrtRSInfo e_emlrtRSI = { 151, /* lineNo */
  "findpeaks",                         /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pathName */
};

static emlrtRSInfo f_emlrtRSI = { 152, /* lineNo */
  "findpeaks",                         /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pathName */
};

static emlrtRSInfo g_emlrtRSI = { 159, /* lineNo */
  "findpeaks",                         /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pathName */
};

static emlrtRSInfo h_emlrtRSI = { 166, /* lineNo */
  "findpeaks",                         /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pathName */
};

static emlrtRSInfo i_emlrtRSI = { 170, /* lineNo */
  "findpeaks",                         /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pathName */
};

static emlrtRSInfo j_emlrtRSI = { 175, /* lineNo */
  "findpeaks",                         /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pathName */
};

static emlrtRSInfo k_emlrtRSI = { 181, /* lineNo */
  "findpeaks",                         /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pathName */
};

static emlrtRSInfo l_emlrtRSI = { 199, /* lineNo */
  "parse_inputs",                      /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pathName */
};

static emlrtRSInfo m_emlrtRSI = { 240, /* lineNo */
  "parse_inputs",                      /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pathName */
};

static emlrtRSInfo n_emlrtRSI = { 337, /* lineNo */
  "parse_inputs",                      /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pathName */
};

static emlrtRSInfo o_emlrtRSI = { 362, /* lineNo */
  "parse_inputs",                      /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pathName */
};

static emlrtRSInfo p_emlrtRSI = { 76,  /* lineNo */
  "validateattributes",                /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/lang/validateattributes.m"/* pathName */
};

static emlrtRSInfo q_emlrtRSI = { 22,  /* lineNo */
  "validatelt",                        /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/+valattr/validatelt.m"/* pathName */
};

static emlrtRSInfo r_emlrtRSI = { 17,  /* lineNo */
  "local_num2str",                     /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/+valattr/private/local_num2str.m"/* pathName */
};

static emlrtRSInfo s_emlrtRSI = { 15,  /* lineNo */
  "num2str",                           /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/num2str.m"/* pathName */
};

static emlrtRSInfo t_emlrtRSI = { 446, /* lineNo */
  "getAllPeaksCodegen",                /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pathName */
};

static emlrtRSInfo v_emlrtRSI = { 532, /* lineNo */
  "removeSmallPeaks",                  /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pathName */
};

static emlrtRSInfo w_emlrtRSI = { 552, /* lineNo */
  "findExtents",                       /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pathName */
};

static emlrtRSInfo x_emlrtRSI = { 555, /* lineNo */
  "findExtents",                       /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pathName */
};

static emlrtRSInfo y_emlrtRSI = { 558, /* lineNo */
  "findExtents",                       /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pathName */
};

static emlrtRSInfo ab_emlrtRSI = { 561,/* lineNo */
  "findExtents",                       /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pathName */
};

static emlrtRSInfo bb_emlrtRSI = { 570,/* lineNo */
  "getPeakBase",                       /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pathName */
};

static emlrtRSInfo cb_emlrtRSI = { 571,/* lineNo */
  "getPeakBase",                       /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pathName */
};

static emlrtRSInfo db_emlrtRSI = { 572,/* lineNo */
  "getPeakBase",                       /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pathName */
};

static emlrtRSInfo eb_emlrtRSI = { 573,/* lineNo */
  "getPeakBase",                       /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pathName */
};

static emlrtRSInfo fb_emlrtRSI = { 574,/* lineNo */
  "getPeakBase",                       /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pathName */
};

static emlrtRSInfo hb_emlrtRSI = { 14, /* lineNo */
  "max",                               /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/datafun/max.m"/* pathName */
};

static emlrtRSInfo ib_emlrtRSI = { 20, /* lineNo */
  "minOrMax",                          /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/minOrMax.m"/* pathName */
};

static emlrtRSInfo jb_emlrtRSI = { 38, /* lineNo */
  "unaryOrBinaryDispatch",             /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/minOrMax.m"/* pathName */
};

static emlrtRSInfo kb_emlrtRSI = { 62, /* lineNo */
  "binaryMinOrMax",                    /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/binaryMinOrMax.m"/* pathName */
};

static emlrtRSInfo lb_emlrtRSI = { 46, /* lineNo */
  "applyBinaryScalarFunction",         /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/applyBinaryScalarFunction.m"/* pathName */
};

static emlrtRSInfo mb_emlrtRSI = { 66, /* lineNo */
  "applyBinaryScalarFunction",         /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/applyBinaryScalarFunction.m"/* pathName */
};

static emlrtRSInfo nb_emlrtRSI = { 202,/* lineNo */
  "flatIter",                          /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/applyBinaryScalarFunction.m"/* pathName */
};

static emlrtRSInfo ob_emlrtRSI = { 662,/* lineNo */
  "removePeaksBelowMinPeakProminence", /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pathName */
};

static emlrtRSInfo pb_emlrtRSI = { 41, /* lineNo */
  "find",                              /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/elmat/find.m"/* pathName */
};

static emlrtRSInfo qb_emlrtRSI = { 153,/* lineNo */
  "eml_find",                          /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/elmat/find.m"/* pathName */
};

static emlrtRSInfo rb_emlrtRSI = { 377,/* lineNo */
  "find_first_indices",                /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/elmat/find.m"/* pathName */
};

static emlrtRSInfo sb_emlrtRSI = { 397,/* lineNo */
  "find_first_indices",                /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/elmat/find.m"/* pathName */
};

static emlrtRSInfo ub_emlrtRSI = { 702,/* lineNo */
  "getPeakWidth",                      /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pathName */
};

static emlrtRSInfo vb_emlrtRSI = { 719,/* lineNo */
  "getHalfMaxBounds",                  /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pathName */
};

static emlrtRSInfo wb_emlrtRSI = { 727,/* lineNo */
  "getHalfMaxBounds",                  /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pathName */
};

static emlrtRSInfo xb_emlrtRSI = { 801,/* lineNo */
  "combineFullPeaks",                  /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pathName */
};

static emlrtRSInfo yb_emlrtRSI = { 804,/* lineNo */
  "combineFullPeaks",                  /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pathName */
};

static emlrtRSInfo ac_emlrtRSI = { 805,/* lineNo */
  "combineFullPeaks",                  /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pathName */
};

static emlrtRSInfo bc_emlrtRSI = { 817,/* lineNo */
  "combineFullPeaks",                  /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pathName */
};

static emlrtRSInfo cc_emlrtRSI = { 818,/* lineNo */
  "combineFullPeaks",                  /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pathName */
};

static emlrtRSInfo dc_emlrtRSI = { 23, /* lineNo */
  "union",                             /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/ops/union.m"/* pathName */
};

static emlrtRSInfo ec_emlrtRSI = { 70, /* lineNo */
  "eml_setop",                         /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/ops/private/eml_setop.m"/* pathName */
};

static emlrtRSInfo kc_emlrtRSI = { 20, /* lineNo */
  "intersect",                         /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/ops/intersect.m"/* pathName */
};

static emlrtRSInfo lc_emlrtRSI = { 174,/* lineNo */
  "flatIter",                          /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/applyBinaryScalarFunction.m"/* pathName */
};

static emlrtRSInfo mc_emlrtRSI = { 14, /* lineNo */
  "min",                               /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/datafun/min.m"/* pathName */
};

static emlrtRSInfo nc_emlrtRSI = { 188,/* lineNo */
  "flatIter",                          /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/applyBinaryScalarFunction.m"/* pathName */
};

static emlrtRSInfo oc_emlrtRSI = { 866,/* lineNo */
  "findPeaksSeparatedByMoreThanMinPeakDistance",/* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pathName */
};

static emlrtRSInfo pc_emlrtRSI = { 878,/* lineNo */
  "findPeaksSeparatedByMoreThanMinPeakDistance",/* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pathName */
};

static emlrtRSInfo qc_emlrtRSI = { 893,/* lineNo */
  "findPeaksSeparatedByMoreThanMinPeakDistance",/* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pathName */
};

static emlrtRSInfo rc_emlrtRSI = { 28, /* lineNo */
  "colon",                             /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/ops/colon.m"/* pathName */
};

static emlrtRSInfo sc_emlrtRSI = { 81, /* lineNo */
  "colon",                             /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/ops/colon.m"/* pathName */
};

static emlrtRSInfo uc_emlrtRSI = { 145,/* lineNo */
  "sortIdx",                           /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/sortIdx.m"/* pathName */
};

static emlrtRSInfo vc_emlrtRSI = { 57, /* lineNo */
  "mergesort",                         /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/mergesort.m"/* pathName */
};

static emlrtRSInfo wc_emlrtRSI = { 112,/* lineNo */
  "mergesort",                         /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/mergesort.m"/* pathName */
};

static emlrtRSInfo xc_emlrtRSI = { 32, /* lineNo */
  "sort",                              /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/datafun/sort.m"/* pathName */
};

static emlrtRSInfo yd_emlrtRSI = { 919,/* lineNo */
  "keepAtMostNpPeaks",                 /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pathName */
};

static emlrtRSInfo ae_emlrtRSI = { 960,/* lineNo */
  "assignFullOutputs",                 /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pathName */
};

static emlrtRSInfo be_emlrtRSI = { 90, /* lineNo */
  "diff",                              /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/datafun/diff.m"/* pathName */
};

static emlrtMCInfo emlrtMCI = { 53,    /* lineNo */
  19,                                  /* colNo */
  "flt2str",                           /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/flt2str.m"/* pName */
};

static emlrtECInfo emlrtECI = { -1,    /* nDims */
  961,                                 /* lineNo */
  7,                                   /* colNo */
  "assignFullOutputs",                 /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtBCInfo m_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  919,                                 /* lineNo */
  13,                                  /* colNo */
  "",                                  /* aName */
  "keepAtMostNpPeaks",                 /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo n_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  919,                                 /* lineNo */
  15,                                  /* colNo */
  "",                                  /* aName */
  "keepAtMostNpPeaks",                 /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo o_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  534,                                 /* lineNo */
  12,                                  /* colNo */
  "",                                  /* aName */
  "removeSmallPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo p_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  536,                                 /* lineNo */
  22,                                  /* colNo */
  "",                                  /* aName */
  "removeSmallPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo q_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  536,                                 /* lineNo */
  31,                                  /* colNo */
  "",                                  /* aName */
  "removeSmallPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo r_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  543,                                 /* lineNo */
  11,                                  /* colNo */
  "",                                  /* aName */
  "removeSmallPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo s_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  543,                                 /* lineNo */
  13,                                  /* colNo */
  "",                                  /* aName */
  "removeSmallPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo t_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  507,                                 /* lineNo */
  11,                                  /* colNo */
  "",                                  /* aName */
  "getAllPeaksCodegen",                /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo u_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  507,                                 /* lineNo */
  13,                                  /* colNo */
  "",                                  /* aName */
  "getAllPeaksCodegen",                /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo v_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  509,                                 /* lineNo */
  21,                                  /* colNo */
  "",                                  /* aName */
  "getAllPeaksCodegen",                /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo w_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  509,                                 /* lineNo */
  23,                                  /* colNo */
  "",                                  /* aName */
  "getAllPeaksCodegen",                /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo x_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  447,                                 /* lineNo */
  10,                                  /* colNo */
  "",                                  /* aName */
  "getAllPeaksCodegen",                /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo y_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  502,                                 /* lineNo */
  49,                                  /* colNo */
  "",                                  /* aName */
  "getAllPeaksCodegen",                /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ab_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  504,                                 /* lineNo */
  5,                                   /* colNo */
  "",                                  /* aName */
  "getAllPeaksCodegen",                /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo bb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  470,                                 /* lineNo */
  17,                                  /* colNo */
  "",                                  /* aName */
  "getAllPeaksCodegen",                /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo cb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  492,                                 /* lineNo */
  17,                                  /* colNo */
  "",                                  /* aName */
  "getAllPeaksCodegen",                /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo db_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  478,                                 /* lineNo */
  17,                                  /* colNo */
  "",                                  /* aName */
  "getAllPeaksCodegen",                /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo eb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  483,                                 /* lineNo */
  21,                                  /* colNo */
  "",                                  /* aName */
  "getAllPeaksCodegen",                /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo fb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  539,                                 /* lineNo */
  13,                                  /* colNo */
  "",                                  /* aName */
  "removeSmallPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo gb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  173,                                 /* lineNo */
  11,                                  /* colNo */
  "",                                  /* aName */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo hb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  924,                                 /* lineNo */
  11,                                  /* colNo */
  "",                                  /* aName */
  "fetchPeakExtents",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ib_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  925,                                 /* lineNo */
  13,                                  /* colNo */
  "",                                  /* aName */
  "fetchPeakExtents",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo jb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  926,                                 /* lineNo */
  13,                                  /* colNo */
  "",                                  /* aName */
  "fetchPeakExtents",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo kb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  927,                                 /* lineNo */
  13,                                  /* colNo */
  "",                                  /* aName */
  "fetchPeakExtents",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo lb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  956,                                 /* lineNo */
  9,                                   /* colNo */
  "",                                  /* aName */
  "assignFullOutputs",                 /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo mb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  957,                                 /* lineNo */
  9,                                   /* colNo */
  "",                                  /* aName */
  "assignFullOutputs",                 /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo nb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  533,                                 /* lineNo */
  9,                                   /* colNo */
  "",                                  /* aName */
  "removeSmallPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtRTEInfo b_emlrtRTEI = { 210,/* lineNo */
  27,                                  /* colNo */
  "parse_inputs",                      /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo c_emlrtRTEI = { 251,/* lineNo */
  31,                                  /* colNo */
  "parse_inputs",                      /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo d_emlrtRTEI = { 13,/* lineNo */
  37,                                  /* colNo */
  "validatenonempty",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/+valattr/validatenonempty.m"/* pName */
};

static emlrtRTEInfo e_emlrtRTEI = { 14,/* lineNo */
  37,                                  /* colNo */
  "validatefinite",                    /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/+valattr/validatefinite.m"/* pName */
};

static emlrtRTEInfo f_emlrtRTEI = { 13,/* lineNo */
  37,                                  /* colNo */
  "validateincreasing",                /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/+valattr/validateincreasing.m"/* pName */
};

static emlrtRTEInfo g_emlrtRTEI = { 22,/* lineNo */
  27,                                  /* colNo */
  "validatelt",                        /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/+valattr/validatelt.m"/* pName */
};

static emlrtBCInfo ob_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  337,                                 /* lineNo */
  83,                                  /* colNo */
  "",                                  /* aName */
  "parse_inputs",                      /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo pb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  601,                                 /* lineNo */
  9,                                   /* colNo */
  "",                                  /* aName */
  "getLeftBase",                       /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo qb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  601,                                 /* lineNo */
  24,                                  /* colNo */
  "",                                  /* aName */
  "getLeftBase",                       /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo rb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  616,                                 /* lineNo */
  7,                                   /* colNo */
  "",                                  /* aName */
  "getLeftBase",                       /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo sb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  602,                                 /* lineNo */
  12,                                  /* colNo */
  "",                                  /* aName */
  "getLeftBase",                       /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo tb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  603,                                 /* lineNo */
  10,                                  /* colNo */
  "",                                  /* aName */
  "getLeftBase",                       /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ub_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  619,                                 /* lineNo */
  16,                                  /* colNo */
  "",                                  /* aName */
  "getLeftBase",                       /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo vb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  620,                                 /* lineNo */
  8,                                   /* colNo */
  "",                                  /* aName */
  "getLeftBase",                       /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo wb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  621,                                 /* lineNo */
  14,                                  /* colNo */
  "",                                  /* aName */
  "getLeftBase",                       /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo xb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  609,                                 /* lineNo */
  20,                                  /* colNo */
  "",                                  /* aName */
  "getLeftBase",                       /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo yb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  622,                                 /* lineNo */
  12,                                  /* colNo */
  "",                                  /* aName */
  "getLeftBase",                       /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ac_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  631,                                 /* lineNo */
  16,                                  /* colNo */
  "",                                  /* aName */
  "getLeftBase",                       /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo bc_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  632,                                 /* lineNo */
  8,                                   /* colNo */
  "",                                  /* aName */
  "getLeftBase",                       /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo cc_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  642,                                 /* lineNo */
  3,                                   /* colNo */
  "",                                  /* aName */
  "getLeftBase",                       /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo dc_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  633,                                 /* lineNo */
  14,                                  /* colNo */
  "",                                  /* aName */
  "getLeftBase",                       /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ec_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  643,                                 /* lineNo */
  3,                                   /* colNo */
  "",                                  /* aName */
  "getLeftBase",                       /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo fc_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  634,                                 /* lineNo */
  12,                                  /* colNo */
  "",                                  /* aName */
  "getLeftBase",                       /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo gc_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  644,                                 /* lineNo */
  3,                                   /* colNo */
  "",                                  /* aName */
  "getLeftBase",                       /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo hc_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  646,                                 /* lineNo */
  6,                                   /* colNo */
  "",                                  /* aName */
  "getLeftBase",                       /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ic_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  646,                                 /* lineNo */
  21,                                  /* colNo */
  "",                                  /* aName */
  "getLeftBase",                       /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo jc_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  647,                                 /* lineNo */
  5,                                   /* colNo */
  "",                                  /* aName */
  "getLeftBase",                       /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo kc_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  648,                                 /* lineNo */
  5,                                   /* colNo */
  "",                                  /* aName */
  "getLeftBase",                       /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtECInfo b_emlrtECI = { -1,  /* nDims */
  827,                                 /* lineNo */
  31,                                  /* colNo */
  "combineFullPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtECInfo c_emlrtECI = { -1,  /* nDims */
  828,                                 /* lineNo */
  31,                                  /* colNo */
  "combineFullPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtECInfo d_emlrtECI = { -1,  /* nDims */
  850,                                 /* lineNo */
  31,                                  /* colNo */
  "combineFullPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtECInfo e_emlrtECI = { -1,  /* nDims */
  851,                                 /* lineNo */
  31,                                  /* colNo */
  "combineFullPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtECInfo f_emlrtECI = { -1,  /* nDims */
  839,                                 /* lineNo */
  1,                                   /* colNo */
  "combineFullPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtECInfo g_emlrtECI = { -1,  /* nDims */
  840,                                 /* lineNo */
  1,                                   /* colNo */
  "combineFullPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtECInfo h_emlrtECI = { -1,  /* nDims */
  841,                                 /* lineNo */
  1,                                   /* colNo */
  "combineFullPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtECInfo i_emlrtECI = { -1,  /* nDims */
  842,                                 /* lineNo */
  1,                                   /* colNo */
  "combineFullPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtECInfo j_emlrtECI = { -1,  /* nDims */
  849,                                 /* lineNo */
  3,                                   /* colNo */
  "combineFullPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtECInfo k_emlrtECI = { -1,  /* nDims */
  850,                                 /* lineNo */
  3,                                   /* colNo */
  "combineFullPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtECInfo l_emlrtECI = { -1,  /* nDims */
  851,                                 /* lineNo */
  3,                                   /* colNo */
  "combineFullPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtECInfo m_emlrtECI = { -1,  /* nDims */
  813,                                 /* lineNo */
  1,                                   /* colNo */
  "combineFullPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtECInfo n_emlrtECI = { -1,  /* nDims */
  825,                                 /* lineNo */
  3,                                   /* colNo */
  "combineFullPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtECInfo o_emlrtECI = { -1,  /* nDims */
  826,                                 /* lineNo */
  3,                                   /* colNo */
  "combineFullPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtECInfo p_emlrtECI = { -1,  /* nDims */
  827,                                 /* lineNo */
  3,                                   /* colNo */
  "combineFullPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtECInfo q_emlrtECI = { -1,  /* nDims */
  828,                                 /* lineNo */
  3,                                   /* colNo */
  "combineFullPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtBCInfo lc_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  813,                                 /* lineNo */
  8,                                   /* colNo */
  "",                                  /* aName */
  "combineFullPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo mc_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  814,                                 /* lineNo */
  8,                                   /* colNo */
  "",                                  /* aName */
  "combineFullPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo nc_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  825,                                 /* lineNo */
  11,                                  /* colNo */
  "",                                  /* aName */
  "combineFullPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo oc_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  825,                                 /* lineNo */
  26,                                  /* colNo */
  "",                                  /* aName */
  "combineFullPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo pc_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  826,                                 /* lineNo */
  11,                                  /* colNo */
  "",                                  /* aName */
  "combineFullPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo qc_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  826,                                 /* lineNo */
  26,                                  /* colNo */
  "",                                  /* aName */
  "combineFullPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo rc_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  827,                                 /* lineNo */
  33,                                  /* colNo */
  "",                                  /* aName */
  "combineFullPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo sc_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  827,                                 /* lineNo */
  41,                                  /* colNo */
  "",                                  /* aName */
  "combineFullPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo tc_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  827,                                 /* lineNo */
  11,                                  /* colNo */
  "",                                  /* aName */
  "combineFullPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo uc_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  828,                                 /* lineNo */
  33,                                  /* colNo */
  "",                                  /* aName */
  "combineFullPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo vc_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  828,                                 /* lineNo */
  41,                                  /* colNo */
  "",                                  /* aName */
  "combineFullPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo wc_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  828,                                 /* lineNo */
  11,                                  /* colNo */
  "",                                  /* aName */
  "combineFullPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo xc_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  839,                                 /* lineNo */
  9,                                   /* colNo */
  "",                                  /* aName */
  "combineFullPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo yc_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  839,                                 /* lineNo */
  24,                                  /* colNo */
  "",                                  /* aName */
  "combineFullPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ad_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  840,                                 /* lineNo */
  9,                                   /* colNo */
  "",                                  /* aName */
  "combineFullPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo bd_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  840,                                 /* lineNo */
  24,                                  /* colNo */
  "",                                  /* aName */
  "combineFullPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo cd_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  841,                                 /* lineNo */
  9,                                   /* colNo */
  "",                                  /* aName */
  "combineFullPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo dd_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  841,                                 /* lineNo */
  26,                                  /* colNo */
  "",                                  /* aName */
  "combineFullPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ed_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  842,                                 /* lineNo */
  9,                                   /* colNo */
  "",                                  /* aName */
  "combineFullPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo fd_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  842,                                 /* lineNo */
  26,                                  /* colNo */
  "",                                  /* aName */
  "combineFullPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo gd_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  849,                                 /* lineNo */
  11,                                  /* colNo */
  "",                                  /* aName */
  "combineFullPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo hd_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  850,                                 /* lineNo */
  33,                                  /* colNo */
  "",                                  /* aName */
  "combineFullPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo id_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  850,                                 /* lineNo */
  41,                                  /* colNo */
  "",                                  /* aName */
  "combineFullPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo jd_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  850,                                 /* lineNo */
  11,                                  /* colNo */
  "",                                  /* aName */
  "combineFullPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo kd_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  851,                                 /* lineNo */
  33,                                  /* colNo */
  "",                                  /* aName */
  "combineFullPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ld_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  851,                                 /* lineNo */
  41,                                  /* colNo */
  "",                                  /* aName */
  "combineFullPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo md_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  851,                                 /* lineNo */
  11,                                  /* colNo */
  "",                                  /* aName */
  "combineFullPeaks",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo nd_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  888,                                 /* lineNo */
  5,                                   /* colNo */
  "",                                  /* aName */
  "findPeaksSeparatedByMoreThanMinPeakDistance",/* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo od_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  893,                                 /* lineNo */
  20,                                  /* colNo */
  "",                                  /* aName */
  "findPeaksSeparatedByMoreThanMinPeakDistance",/* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo pd_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  872,                                 /* lineNo */
  8,                                   /* colNo */
  "",                                  /* aName */
  "findPeaksSeparatedByMoreThanMinPeakDistance",/* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo qd_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  884,                                 /* lineNo */
  7,                                   /* colNo */
  "",                                  /* aName */
  "findPeaksSeparatedByMoreThanMinPeakDistance",/* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo rd_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  880,                                 /* lineNo */
  18,                                  /* colNo */
  "",                                  /* aName */
  "findPeaksSeparatedByMoreThanMinPeakDistance",/* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo sd_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  872,                                 /* lineNo */
  10,                                  /* colNo */
  "",                                  /* aName */
  "findPeaksSeparatedByMoreThanMinPeakDistance",/* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo td_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  871,                                 /* lineNo */
  9,                                   /* colNo */
  "",                                  /* aName */
  "findPeaksSeparatedByMoreThanMinPeakDistance",/* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtECInfo r_emlrtECI = { -1,  /* nDims */
  887,                                 /* lineNo */
  25,                                  /* colNo */
  "findPeaksSeparatedByMoreThanMinPeakDistance",/* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtECInfo s_emlrtECI = { -1,  /* nDims */
  887,                                 /* lineNo */
  15,                                  /* colNo */
  "findPeaksSeparatedByMoreThanMinPeakDistance",/* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtBCInfo ud_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  731,                                 /* lineNo */
  5,                                   /* colNo */
  "",                                  /* aName */
  "getHalfMaxBounds",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo vd_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  731,                                 /* lineNo */
  77,                                  /* colNo */
  "",                                  /* aName */
  "getHalfMaxBounds",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo wd_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  729,                                 /* lineNo */
  5,                                   /* colNo */
  "",                                  /* aName */
  "getHalfMaxBounds",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo xd_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  729,                                 /* lineNo */
  19,                                  /* colNo */
  "",                                  /* aName */
  "getHalfMaxBounds",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo yd_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  728,                                 /* lineNo */
  15,                                  /* colNo */
  "",                                  /* aName */
  "getHalfMaxBounds",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ae_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  723,                                 /* lineNo */
  5,                                   /* colNo */
  "",                                  /* aName */
  "getHalfMaxBounds",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo be_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  748,                                 /* lineNo */
  25,                                  /* colNo */
  "",                                  /* aName */
  "findRightIntercept",                /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ce_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  723,                                 /* lineNo */
  69,                                  /* colNo */
  "",                                  /* aName */
  "getHalfMaxBounds",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo de_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  727,                                 /* lineNo */
  34,                                  /* colNo */
  "",                                  /* aName */
  "getHalfMaxBounds",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ee_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  721,                                 /* lineNo */
  5,                                   /* colNo */
  "",                                  /* aName */
  "getHalfMaxBounds",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo fe_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  721,                                 /* lineNo */
  19,                                  /* colNo */
  "",                                  /* aName */
  "getHalfMaxBounds",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ge_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  720,                                 /* lineNo */
  14,                                  /* colNo */
  "",                                  /* aName */
  "getHalfMaxBounds",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo he_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  740,                                 /* lineNo */
  25,                                  /* colNo */
  "",                                  /* aName */
  "findLeftIntercept",                 /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ie_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  719,                                 /* lineNo */
  32,                                  /* colNo */
  "",                                  /* aName */
  "getHalfMaxBounds",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo je_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  716,                                 /* lineNo */
  26,                                  /* colNo */
  "",                                  /* aName */
  "getHalfMaxBounds",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ke_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  716,                                 /* lineNo */
  16,                                  /* colNo */
  "",                                  /* aName */
  "getHalfMaxBounds",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo le_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  666,                                 /* lineNo */
  11,                                  /* colNo */
  "",                                  /* aName */
  "removePeaksBelowMinPeakProminence", /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo me_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  665,                                 /* lineNo */
  11,                                  /* colNo */
  "",                                  /* aName */
  "removePeaksBelowMinPeakProminence", /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ne_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  664,                                 /* lineNo */
  13,                                  /* colNo */
  "",                                  /* aName */
  "removePeaksBelowMinPeakProminence", /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo oe_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  663,                                 /* lineNo */
  11,                                  /* colNo */
  "",                                  /* aName */
  "removePeaksBelowMinPeakProminence", /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo pe_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  659,                                 /* lineNo */
  9,                                   /* colNo */
  "",                                  /* aName */
  "removePeaksBelowMinPeakProminence", /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo qe_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  574,                                 /* lineNo */
  39,                                  /* colNo */
  "",                                  /* aName */
  "getPeakBase",                       /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo re_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  574,                                 /* lineNo */
  22,                                  /* colNo */
  "",                                  /* aName */
  "getPeakBase",                       /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo se_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  549,                                 /* lineNo */
  1,                                   /* colNo */
  "",                                  /* aName */
  "findExtents",                       /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtRTEInfo p_emlrtRTEI = { 19,/* lineNo */
  23,                                  /* colNo */
  "scalexpAlloc",                      /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/scalexpAlloc.m"/* pName */
};

static emlrtECInfo t_emlrtECI = { -1,  /* nDims */
  659,                                 /* lineNo */
  7,                                   /* colNo */
  "removePeaksBelowMinPeakProminence", /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo q_emlrtRTEI = { 387,/* lineNo */
  1,                                   /* colNo */
  "find_first_indices",                /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/elmat/find.m"/* pName */
};

static emlrtBCInfo te_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  719,                                 /* lineNo */
  44,                                  /* colNo */
  "",                                  /* aName */
  "getHalfMaxBounds",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ue_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  723,                                 /* lineNo */
  29,                                  /* colNo */
  "",                                  /* aName */
  "getHalfMaxBounds",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ve_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  723,                                 /* lineNo */
  38,                                  /* colNo */
  "",                                  /* aName */
  "getHalfMaxBounds",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo we_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  723,                                 /* lineNo */
  49,                                  /* colNo */
  "",                                  /* aName */
  "getHalfMaxBounds",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo xe_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  723,                                 /* lineNo */
  58,                                  /* colNo */
  "",                                  /* aName */
  "getHalfMaxBounds",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ye_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  723,                                 /* lineNo */
  82,                                  /* colNo */
  "",                                  /* aName */
  "getHalfMaxBounds",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo af_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  727,                                 /* lineNo */
  46,                                  /* colNo */
  "",                                  /* aName */
  "getHalfMaxBounds",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo bf_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  731,                                 /* lineNo */
  29,                                  /* colNo */
  "",                                  /* aName */
  "getHalfMaxBounds",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo cf_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  731,                                 /* lineNo */
  40,                                  /* colNo */
  "",                                  /* aName */
  "getHalfMaxBounds",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo df_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  731,                                 /* lineNo */
  53,                                  /* colNo */
  "",                                  /* aName */
  "getHalfMaxBounds",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ef_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  731,                                 /* lineNo */
  64,                                  /* colNo */
  "",                                  /* aName */
  "getHalfMaxBounds",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ff_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  731,                                 /* lineNo */
  90,                                  /* colNo */
  "",                                  /* aName */
  "getHalfMaxBounds",                  /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m",/* pName */
  0                                    /* checkKind */
};

static emlrtRTEInfo y_emlrtRTEI = { 151,/* lineNo */
  5,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo ab_emlrtRTEI = { 507,/* lineNo */
  1,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo bb_emlrtRTEI = { 509,/* lineNo */
  1,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo cb_emlrtRTEI = { 152,/* lineNo */
  11,                                  /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo db_emlrtRTEI = { 152,/* lineNo */
  5,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo eb_emlrtRTEI = { 159,/* lineNo */
  4,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo fb_emlrtRTEI = { 173,/* lineNo */
  1,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo gb_emlrtRTEI = { 924,/* lineNo */
  11,                                  /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo hb_emlrtRTEI = { 170,/* lineNo */
  1,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo ib_emlrtRTEI = { 924,/* lineNo */
  1,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo jb_emlrtRTEI = { 927,/* lineNo */
  13,                                  /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo kb_emlrtRTEI = { 965,/* lineNo */
  3,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo lb_emlrtRTEI = { 974,/* lineNo */
  3,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo mb_emlrtRTEI = { 147,/* lineNo */
  5,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo nb_emlrtRTEI = { 166,/* lineNo */
  1,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo ob_emlrtRTEI = { 135,/* lineNo */
  2,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo pb_emlrtRTEI = { 135,/* lineNo */
  11,                                  /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo qb_emlrtRTEI = { 1,/* lineNo */
  30,                                  /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo rb_emlrtRTEI = { 159,/* lineNo */
  8,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo sb_emlrtRTEI = { 159,/* lineNo */
  12,                                  /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo tb_emlrtRTEI = { 159,/* lineNo */
  17,                                  /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo ub_emlrtRTEI = { 159,/* lineNo */
  22,                                  /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo vb_emlrtRTEI = { 201,/* lineNo */
  1,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo wb_emlrtRTEI = { 255,/* lineNo */
  5,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo xb_emlrtRTEI = { 582,/* lineNo */
  1,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo yb_emlrtRTEI = { 583,/* lineNo */
  1,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo ac_emlrtRTEI = { 586,/* lineNo */
  1,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo bc_emlrtRTEI = { 587,/* lineNo */
  1,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo cc_emlrtRTEI = { 588,/* lineNo */
  1,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo dc_emlrtRTEI = { 21,/* lineNo */
  5,                                   /* colNo */
  "intersect",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/ops/intersect.m"/* pName */
};

static emlrtRTEInfo ec_emlrtRTEI = { 812,/* lineNo */
  1,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo fc_emlrtRTEI = { 813,/* lineNo */
  8,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo gc_emlrtRTEI = { 814,/* lineNo */
  8,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo hc_emlrtRTEI = { 817,/* lineNo */
  15,                                  /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo ic_emlrtRTEI = { 62,/* lineNo */
  10,                                  /* colNo */
  "binaryMinOrMax",                    /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/binaryMinOrMax.m"/* pName */
};

static emlrtRTEInfo jc_emlrtRTEI = { 818,/* lineNo */
  13,                                  /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo kc_emlrtRTEI = { 824,/* lineNo */
  3,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo lc_emlrtRTEI = { 825,/* lineNo */
  11,                                  /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo mc_emlrtRTEI = { 826,/* lineNo */
  11,                                  /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo nc_emlrtRTEI = { 827,/* lineNo */
  11,                                  /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo oc_emlrtRTEI = { 827,/* lineNo */
  26,                                  /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo pc_emlrtRTEI = { 828,/* lineNo */
  11,                                  /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo qc_emlrtRTEI = { 828,/* lineNo */
  26,                                  /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo rc_emlrtRTEI = { 838,/* lineNo */
  1,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo sc_emlrtRTEI = { 839,/* lineNo */
  9,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo tc_emlrtRTEI = { 840,/* lineNo */
  9,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo uc_emlrtRTEI = { 841,/* lineNo */
  9,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo vc_emlrtRTEI = { 842,/* lineNo */
  9,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo wc_emlrtRTEI = { 848,/* lineNo */
  3,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo xc_emlrtRTEI = { 849,/* lineNo */
  11,                                  /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo yc_emlrtRTEI = { 850,/* lineNo */
  11,                                  /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo ad_emlrtRTEI = { 850,/* lineNo */
  26,                                  /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo bd_emlrtRTEI = { 851,/* lineNo */
  11,                                  /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo cd_emlrtRTEI = { 851,/* lineNo */
  26,                                  /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo dd_emlrtRTEI = { 817,/* lineNo */
  1,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo ed_emlrtRTEI = { 818,/* lineNo */
  1,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo fd_emlrtRTEI = { 800,/* lineNo */
  52,                                  /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo kd_emlrtRTEI = { 878,/* lineNo */
  5,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo ld_emlrtRTEI = { 145,/* lineNo */
  23,                                  /* colNo */
  "sortIdx",                           /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/sortIdx.m"/* pName */
};

static emlrtRTEInfo md_emlrtRTEI = { 880,/* lineNo */
  1,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo nd_emlrtRTEI = { 882,/* lineNo */
  1,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo od_emlrtRTEI = { 860,/* lineNo */
  16,                                  /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo pd_emlrtRTEI = { 887,/* lineNo */
  26,                                  /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo qd_emlrtRTEI = { 893,/* lineNo */
  1,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo rd_emlrtRTEI = { 887,/* lineNo */
  55,                                  /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo sd_emlrtRTEI = { 887,/* lineNo */
  15,                                  /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo td_emlrtRTEI = { 876,/* lineNo */
  9,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo ud_emlrtRTEI = { 893,/* lineNo */
  20,                                  /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo vd_emlrtRTEI = { 52,/* lineNo */
  1,                                   /* colNo */
  "mergesort",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/mergesort.m"/* pName */
};

static emlrtRTEInfo ae_emlrtRTEI = { 548,/* lineNo */
  1,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo be_emlrtRTEI = { 571,/* lineNo */
  48,                                  /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo ce_emlrtRTEI = { 571,/* lineNo */
  60,                                  /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo de_emlrtRTEI = { 571,/* lineNo */
  73,                                  /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo ee_emlrtRTEI = { 46,/* lineNo */
  6,                                   /* colNo */
  "applyBinaryScalarFunction",         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/applyBinaryScalarFunction.m"/* pName */
};

static emlrtRTEInfo fe_emlrtRTEI = { 574,/* lineNo */
  16,                                  /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo ge_emlrtRTEI = { 574,/* lineNo */
  33,                                  /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo he_emlrtRTEI = { 662,/* lineNo */
  12,                                  /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo ie_emlrtRTEI = { 153,/* lineNo */
  13,                                  /* colNo */
  "find",                              /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/elmat/find.m"/* pName */
};

static emlrtRTEInfo je_emlrtRTEI = { 41,/* lineNo */
  5,                                   /* colNo */
  "find",                              /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/elmat/find.m"/* pName */
};

static emlrtRTEInfo ke_emlrtRTEI = { 662,/* lineNo */
  1,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo le_emlrtRTEI = { 663,/* lineNo */
  7,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo me_emlrtRTEI = { 663,/* lineNo */
  1,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo ne_emlrtRTEI = { 664,/* lineNo */
  8,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo oe_emlrtRTEI = { 664,/* lineNo */
  1,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo pe_emlrtRTEI = { 665,/* lineNo */
  7,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo qe_emlrtRTEI = { 665,/* lineNo */
  1,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo re_emlrtRTEI = { 666,/* lineNo */
  7,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo se_emlrtRTEI = { 666,/* lineNo */
  1,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo te_emlrtRTEI = { 694,/* lineNo */
  3,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo ue_emlrtRTEI = { 702,/* lineNo */
  1,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo ve_emlrtRTEI = { 555,/* lineNo */
  2,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo we_emlrtRTEI = { 66,/* lineNo */
  27,                                  /* colNo */
  "applyBinaryScalarFunction",         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/applyBinaryScalarFunction.m"/* pName */
};

static emlrtRTEInfo xe_emlrtRTEI = { 546,/* lineNo */
  37,                                  /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo ye_emlrtRTEI = { 571,/* lineNo */
  2,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo af_emlrtRTEI = { 552,/* lineNo */
  6,                                   /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo bf_emlrtRTEI = { 552,/* lineNo */
  10,                                  /* colNo */
  "findpeaks",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/signal/signal/findpeaks.m"/* pName */
};

static emlrtRTEInfo cf_emlrtRTEI = { 33,/* lineNo */
  6,                                   /* colNo */
  "find",                              /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/elmat/find.m"/* pName */
};

static emlrtRSInfo me_emlrtRSI = { 53, /* lineNo */
  "flt2str",                           /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/eml/+coder/+internal/flt2str.m"/* pathName */
};

/* Function Declarations */
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, char_T y[23]);
static const mxArray *b_sprintf(const emlrtStack *sp, const mxArray *b, const
  mxArray *c, emlrtMCInfo *location);
static void c_findPeaksSeparatedByMoreThanM(const emlrtStack *sp, const
  emxArray_real_T *y, const emxArray_real_T *x, const emxArray_int32_T *iPk,
  emxArray_int32_T *idx);
static void combineFullPeaks(const emlrtStack *sp, const emxArray_real_T *y,
  const emxArray_real_T *x, const emxArray_int32_T *iPk, const emxArray_real_T
  *bPk, const emxArray_int32_T *iLBw, const emxArray_int32_T *iRBw, const
  emxArray_real_T *wPk, const emxArray_int32_T *iInf, emxArray_int32_T *iPkOut,
  emxArray_real_T *bPkOut, emxArray_real_T *bxPkOut, emxArray_real_T *byPkOut,
  emxArray_real_T *wxPkOut);
static void emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *a__output_of_sprintf_, const char_T *identifier, char_T y[23]);
static void findExtents(const emlrtStack *sp, const emxArray_real_T *y, const
  emxArray_real_T *x, emxArray_int32_T *iPk, const emxArray_int32_T *iFin, const
  emxArray_int32_T *iInf, const emxArray_int32_T *iInflect, emxArray_real_T *bPk,
  emxArray_real_T *bxPk, emxArray_real_T *byPk, emxArray_real_T *wxPk);
static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, char_T ret[23]);
static void getLeftBase(const emlrtStack *sp, const emxArray_real_T *yTemp,
  const emxArray_int32_T *iPeak, const emxArray_int32_T *iFinite, const
  emxArray_int32_T *iInflect, emxArray_int32_T *iBase, emxArray_int32_T *iSaddle);
static void parse_inputs(const emlrtStack *sp, const emxArray_real_T *Yin, const
  emxArray_real_T *varargin_1, emxArray_real_T *y, emxArray_real_T *x, real_T
  *NpOut);

/* Function Definitions */
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, char_T y[23])
{
  g_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static const mxArray *b_sprintf(const emlrtStack *sp, const mxArray *b, const
  mxArray *c, emlrtMCInfo *location)
{
  const mxArray *pArrays[2];
  const mxArray *m;
  pArrays[0] = b;
  pArrays[1] = c;
  return emlrtCallMATLABR2012b(sp, 1, &m, 2, pArrays, "sprintf", true, location);
}

static void c_findPeaksSeparatedByMoreThanM(const emlrtStack *sp, const
  emxArray_real_T *y, const emxArray_real_T *x, const emxArray_int32_T *iPk,
  emxArray_int32_T *idx)
{
  int32_T i2;
  int32_T i;
  emxArray_int32_T *b_sortIdx;
  int32_T n;
  emxArray_int32_T *iwork;
  int32_T k;
  int32_T b_i;
  emxArray_real_T *locs_temp;
  int32_T j;
  int32_T pEnd;
  int32_T p;
  int32_T q;
  int32_T qEnd;
  emxArray_boolean_T *idelete;
  int32_T kEnd;
  int32_T i1;
  emxArray_boolean_T *r;
  emxArray_boolean_T *b_idelete;
  emxArray_int32_T *r1;
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
  if (iPk->size[0] == 0) {
    st.site = &oc_emlrtRSI;
    b_st.site = &rc_emlrtRSI;
    c_st.site = &sc_emlrtRSI;
    idx->size[0] = 0;
  } else {
    i2 = iPk->size[0];
    for (i = 0; i < i2; i++) {
      if ((iPk->data[i] < 1) || (iPk->data[i] > y->size[0])) {
        emlrtDynamicBoundsCheckR2012b(iPk->data[i], 1, y->size[0], &td_emlrtBCI,
          sp);
      }
    }

    i2 = iPk->size[0];
    for (i = 0; i < i2; i++) {
      if ((iPk->data[i] < 1) || (iPk->data[i] > x->size[0])) {
        emlrtDynamicBoundsCheckR2012b(iPk->data[i], 1, x->size[0], &sd_emlrtBCI,
          sp);
      }
    }

    emxInit_int32_T(sp, &b_sortIdx, 1, &td_emlrtRTEI, true);
    st.site = &pc_emlrtRSI;
    n = iPk->size[0] + 1;
    i = b_sortIdx->size[0];
    b_sortIdx->size[0] = iPk->size[0];
    emxEnsureCapacity_int32_T(&st, b_sortIdx, i, &kd_emlrtRTEI);
    i2 = iPk->size[0];
    for (i = 0; i < i2; i++) {
      b_sortIdx->data[i] = 0;
    }

    emxInit_int32_T(&st, &iwork, 1, &vd_emlrtRTEI, true);
    b_st.site = &uc_emlrtRSI;
    i = iwork->size[0];
    iwork->size[0] = iPk->size[0];
    emxEnsureCapacity_int32_T(&b_st, iwork, i, &ld_emlrtRTEI);
    i2 = iPk->size[0] - 1;
    c_st.site = &vc_emlrtRSI;
    if ((1 <= iPk->size[0] - 1) && (iPk->size[0] - 1 > 2147483645)) {
      d_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&d_st);
    }

    for (k = 1; k <= i2; k += 2) {
      i = iPk->data[k - 1] - 1;
      if ((y->data[i] >= y->data[iPk->data[k] - 1]) || muDoubleScalarIsNaN
          (y->data[i])) {
        b_sortIdx->data[k - 1] = k;
        b_sortIdx->data[k] = k + 1;
      } else {
        b_sortIdx->data[k - 1] = k + 1;
        b_sortIdx->data[k] = k;
      }
    }

    if ((iPk->size[0] & 1) != 0) {
      b_sortIdx->data[iPk->size[0] - 1] = iPk->size[0];
    }

    b_i = 2;
    while (b_i < n - 1) {
      i2 = b_i << 1;
      j = 1;
      for (pEnd = b_i + 1; pEnd < n; pEnd = qEnd + b_i) {
        p = j - 1;
        q = pEnd;
        qEnd = j + i2;
        if (qEnd > n) {
          qEnd = n;
        }

        k = 0;
        kEnd = qEnd - j;
        while (k + 1 <= kEnd) {
          i = iPk->data[b_sortIdx->data[p] - 1] - 1;
          i1 = b_sortIdx->data[q - 1];
          if ((y->data[i] >= y->data[iPk->data[i1 - 1] - 1]) ||
              muDoubleScalarIsNaN(y->data[i])) {
            iwork->data[k] = b_sortIdx->data[p];
            p++;
            if (p + 1 == pEnd) {
              while (q < qEnd) {
                k++;
                iwork->data[k] = b_sortIdx->data[q - 1];
                q++;
              }
            }
          } else {
            iwork->data[k] = i1;
            q++;
            if (q == qEnd) {
              while (p + 1 < pEnd) {
                k++;
                iwork->data[k] = b_sortIdx->data[p];
                p++;
              }
            }
          }

          k++;
        }

        c_st.site = &wc_emlrtRSI;
        for (k = 0; k < kEnd; k++) {
          b_sortIdx->data[(j + k) - 1] = iwork->data[k];
        }

        j = qEnd;
      }

      b_i = i2;
    }

    emxFree_int32_T(&iwork);
    emxInit_real_T(&b_st, &locs_temp, 1, &md_emlrtRTEI, true);
    i = locs_temp->size[0];
    locs_temp->size[0] = b_sortIdx->size[0];
    emxEnsureCapacity_real_T(sp, locs_temp, i, &md_emlrtRTEI);
    i2 = b_sortIdx->size[0];
    for (i = 0; i < i2; i++) {
      if ((b_sortIdx->data[i] < 1) || (b_sortIdx->data[i] > iPk->size[0])) {
        emlrtDynamicBoundsCheckR2012b(b_sortIdx->data[i], 1, iPk->size[0],
          &rd_emlrtBCI, sp);
      }

      locs_temp->data[i] = x->data[iPk->data[b_sortIdx->data[i] - 1] - 1];
    }

    emxInit_boolean_T(sp, &idelete, 1, &nd_emlrtRTEI, true);
    i = idelete->size[0];
    idelete->size[0] = b_sortIdx->size[0];
    emxEnsureCapacity_boolean_T(sp, idelete, i, &nd_emlrtRTEI);
    i2 = b_sortIdx->size[0];
    for (i = 0; i < i2; i++) {
      idelete->data[i] = false;
    }

    i = b_sortIdx->size[0];
    emxInit_boolean_T(sp, &r, 1, &od_emlrtRTEI, true);
    emxInit_boolean_T(sp, &b_idelete, 1, &od_emlrtRTEI, true);
    for (b_i = 0; b_i < i; b_i++) {
      i1 = b_i + 1;
      if ((i1 < 1) || (i1 > idelete->size[0])) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, idelete->size[0], &qd_emlrtBCI, sp);
      }

      if (!idelete->data[i1 - 1]) {
        i1 = b_i + 1;
        if ((i1 < 1) || (i1 > b_sortIdx->size[0])) {
          emlrtDynamicBoundsCheckR2012b(i1, 1, b_sortIdx->size[0], &pd_emlrtBCI,
            sp);
        }

        i2 = locs_temp->size[0];
        i1 = r->size[0];
        r->size[0] = locs_temp->size[0];
        emxEnsureCapacity_boolean_T(sp, r, i1, &pd_emlrtRTEI);
        for (i1 = 0; i1 < i2; i1++) {
          r->data[i1] = (locs_temp->data[i1] >= x->data[iPk->data
                         [b_sortIdx->data[b_i] - 1] - 1] - 50.0);
        }

        i1 = b_i + 1;
        if ((i1 < 1) || (i1 > b_sortIdx->size[0])) {
          emlrtDynamicBoundsCheckR2012b(i1, 1, b_sortIdx->size[0], &pd_emlrtBCI,
            sp);
        }

        i2 = locs_temp->size[0];
        i1 = b_idelete->size[0];
        b_idelete->size[0] = locs_temp->size[0];
        emxEnsureCapacity_boolean_T(sp, b_idelete, i1, &rd_emlrtRTEI);
        for (i1 = 0; i1 < i2; i1++) {
          b_idelete->data[i1] = (locs_temp->data[i1] <= x->data[iPk->
            data[b_sortIdx->data[b_i] - 1] - 1] + 50.0);
        }

        i2 = r->size[0];
        if (r->size[0] != b_idelete->size[0]) {
          emlrtSizeEqCheck1DR2012b(r->size[0], b_idelete->size[0], &r_emlrtECI,
            sp);
        }

        for (i1 = 0; i1 < i2; i1++) {
          r->data[i1] = (r->data[i1] && b_idelete->data[i1]);
        }

        if (idelete->size[0] != r->size[0]) {
          emlrtSizeEqCheck1DR2012b(idelete->size[0], r->size[0], &s_emlrtECI, sp);
        }

        i1 = b_idelete->size[0];
        b_idelete->size[0] = idelete->size[0];
        emxEnsureCapacity_boolean_T(sp, b_idelete, i1, &sd_emlrtRTEI);
        i2 = idelete->size[0];
        for (i1 = 0; i1 < i2; i1++) {
          b_idelete->data[i1] = idelete->data[i1];
          idelete->data[i1] = (idelete->data[i1] || r->data[i1]);
        }

        i1 = b_i + 1;
        if ((i1 < 1) || (i1 > b_idelete->size[0])) {
          emlrtDynamicBoundsCheckR2012b(i1, 1, b_idelete->size[0], &nd_emlrtBCI,
            sp);
        }

        idelete->data[i1 - 1] = false;
      }
    }

    emxFree_boolean_T(&b_idelete);
    emxFree_boolean_T(&r);
    emxFree_real_T(&locs_temp);
    j = idelete->size[0] - 1;
    i2 = 0;
    for (b_i = 0; b_i <= j; b_i++) {
      if (!idelete->data[b_i]) {
        i2++;
      }
    }

    emxInit_int32_T(sp, &r1, 1, &ud_emlrtRTEI, true);
    i = r1->size[0];
    r1->size[0] = i2;
    emxEnsureCapacity_int32_T(sp, r1, i, &od_emlrtRTEI);
    i2 = 0;
    for (b_i = 0; b_i <= j; b_i++) {
      if (!idelete->data[b_i]) {
        r1->data[i2] = b_i + 1;
        i2++;
      }
    }

    emxFree_boolean_T(&idelete);
    st.site = &qc_emlrtRSI;
    i2 = r1->size[0];
    for (i = 0; i < i2; i++) {
      if ((r1->data[i] < 1) || (r1->data[i] > b_sortIdx->size[0])) {
        emlrtDynamicBoundsCheckR2012b(r1->data[i], 1, b_sortIdx->size[0],
          &od_emlrtBCI, &st);
      }
    }

    i = idx->size[0];
    idx->size[0] = r1->size[0];
    emxEnsureCapacity_int32_T(&st, idx, i, &qd_emlrtRTEI);
    i2 = r1->size[0];
    for (i = 0; i < i2; i++) {
      idx->data[i] = b_sortIdx->data[r1->data[i] - 1];
    }

    emxFree_int32_T(&r1);
    emxFree_int32_T(&b_sortIdx);
    b_st.site = &xc_emlrtRSI;
    sort(&b_st, idx);
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

static void combineFullPeaks(const emlrtStack *sp, const emxArray_real_T *y,
  const emxArray_real_T *x, const emxArray_int32_T *iPk, const emxArray_real_T
  *bPk, const emxArray_int32_T *iLBw, const emxArray_int32_T *iRBw, const
  emxArray_real_T *wPk, const emxArray_int32_T *iInf, emxArray_int32_T *iPkOut,
  emxArray_real_T *bPkOut, emxArray_real_T *bxPkOut, emxArray_real_T *byPkOut,
  emxArray_real_T *wxPkOut)
{
  emxArray_int32_T *iFinite;
  emxArray_int32_T *ia;
  emxArray_int32_T *ib;
  emxArray_int32_T *c;
  int32_T nx;
  int32_T loop_ub;
  emxArray_int32_T *iInfL;
  int32_T varargin_1;
  emxArray_int32_T *iInfR;
  int32_T b_ia[1];
  emxArray_real_T *r;
  int32_T c_ia[2];
  emlrtStack st;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack h_st;
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
  f_st.prev = &e_st;
  f_st.tls = e_st.tls;
  g_st.prev = &f_st;
  g_st.tls = f_st.tls;
  h_st.prev = &g_st;
  h_st.tls = g_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  emxInit_int32_T(sp, &iFinite, 1, &fd_emlrtRTEI, true);
  emxInit_int32_T(sp, &ia, 1, &hc_emlrtRTEI, true);
  emxInit_int32_T(sp, &ib, 1, &fd_emlrtRTEI, true);
  emxInit_int32_T(sp, &c, 1, &fd_emlrtRTEI, true);
  st.site = &xb_emlrtRSI;
  b_st.site = &dc_emlrtRSI;
  c_st.site = &ec_emlrtRSI;
  do_vectors(&c_st, iPk, iInf, iPkOut, ia, ib);
  st.site = &yb_emlrtRSI;
  b_st.site = &kc_emlrtRSI;
  c_st.site = &ec_emlrtRSI;
  b_do_vectors(&c_st, iPkOut, iPk, c, ia, ib);
  nx = iFinite->size[0];
  iFinite->size[0] = ia->size[0];
  emxEnsureCapacity_int32_T(&st, iFinite, nx, &dc_emlrtRTEI);
  loop_ub = ia->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    iFinite->data[nx] = ia->data[nx];
  }

  st.site = &ac_emlrtRSI;
  b_st.site = &kc_emlrtRSI;
  c_st.site = &ec_emlrtRSI;
  b_do_vectors(&c_st, iPkOut, iInf, c, ia, ib);
  nx = ib->size[0];
  ib->size[0] = ia->size[0];
  emxEnsureCapacity_int32_T(&st, ib, nx, &dc_emlrtRTEI);
  loop_ub = ia->size[0];
  emxFree_int32_T(&c);
  for (nx = 0; nx < loop_ub; nx++) {
    ib->data[nx] = ia->data[nx];
  }

  nx = bPkOut->size[0];
  bPkOut->size[0] = iPkOut->size[0];
  emxEnsureCapacity_real_T(sp, bPkOut, nx, &ec_emlrtRTEI);
  loop_ub = iPkOut->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    bPkOut->data[nx] = 0.0;
  }

  nx = ia->size[0];
  ia->size[0] = iFinite->size[0];
  emxEnsureCapacity_int32_T(sp, ia, nx, &fc_emlrtRTEI);
  loop_ub = iFinite->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    if ((iFinite->data[nx] < 1) || (iFinite->data[nx] > iPkOut->size[0])) {
      emlrtDynamicBoundsCheckR2012b(iFinite->data[nx], 1, iPkOut->size[0],
        &lc_emlrtBCI, sp);
    }

    ia->data[nx] = iFinite->data[nx];
  }

  if (ia->size[0] != bPk->size[0]) {
    emlrtSubAssignSizeCheck1dR2017a(ia->size[0], bPk->size[0], &m_emlrtECI, sp);
  }

  loop_ub = bPk->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    bPkOut->data[ia->data[nx] - 1] = bPk->data[nx];
  }

  nx = ia->size[0];
  ia->size[0] = ib->size[0];
  emxEnsureCapacity_int32_T(sp, ia, nx, &gc_emlrtRTEI);
  loop_ub = ib->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    if ((ib->data[nx] < 1) || (ib->data[nx] > bPkOut->size[0])) {
      emlrtDynamicBoundsCheckR2012b(ib->data[nx], 1, bPkOut->size[0],
        &mc_emlrtBCI, sp);
    }

    ia->data[nx] = ib->data[nx];
  }

  loop_ub = ia->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    bPkOut->data[ia->data[nx] - 1] = 0.0;
  }

  st.site = &bc_emlrtRSI;
  nx = ia->size[0];
  ia->size[0] = iInf->size[0];
  emxEnsureCapacity_int32_T(&st, ia, nx, &hc_emlrtRTEI);
  loop_ub = iInf->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    ia->data[nx] = iInf->data[nx] - 1;
  }

  emxInit_int32_T(&st, &iInfL, 1, &dd_emlrtRTEI, true);
  b_st.site = &hb_emlrtRSI;
  c_st.site = &ib_emlrtRSI;
  d_st.site = &jb_emlrtRSI;
  e_st.site = &kb_emlrtRSI;
  nx = iInfL->size[0];
  iInfL->size[0] = ia->size[0];
  emxEnsureCapacity_int32_T(&e_st, iInfL, nx, &ic_emlrtRTEI);
  f_st.site = &mb_emlrtRSI;
  nx = ia->size[0];
  g_st.site = &lc_emlrtRSI;
  if ((1 <= ia->size[0]) && (ia->size[0] > 2147483646)) {
    h_st.site = &u_emlrtRSI;
    check_forloop_overflow_error(&h_st);
  }

  for (loop_ub = 0; loop_ub < nx; loop_ub++) {
    if (1 < ia->data[loop_ub]) {
      iInfL->data[loop_ub] = ia->data[loop_ub];
    } else {
      iInfL->data[loop_ub] = 1;
    }
  }

  varargin_1 = x->size[0];
  st.site = &cc_emlrtRSI;
  nx = ia->size[0];
  ia->size[0] = iInf->size[0];
  emxEnsureCapacity_int32_T(&st, ia, nx, &jc_emlrtRTEI);
  loop_ub = iInf->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    ia->data[nx] = iInf->data[nx] + 1;
  }

  emxInit_int32_T(&st, &iInfR, 1, &ed_emlrtRTEI, true);
  b_st.site = &mc_emlrtRSI;
  c_st.site = &ib_emlrtRSI;
  d_st.site = &jb_emlrtRSI;
  e_st.site = &kb_emlrtRSI;
  nx = iInfR->size[0];
  iInfR->size[0] = ia->size[0];
  emxEnsureCapacity_int32_T(&e_st, iInfR, nx, &ic_emlrtRTEI);
  f_st.site = &mb_emlrtRSI;
  nx = ia->size[0];
  g_st.site = &nc_emlrtRSI;
  if ((1 <= ia->size[0]) && (ia->size[0] > 2147483646)) {
    h_st.site = &u_emlrtRSI;
    check_forloop_overflow_error(&h_st);
  }

  for (loop_ub = 0; loop_ub < nx; loop_ub++) {
    if (ia->data[loop_ub] > varargin_1) {
      iInfR->data[loop_ub] = varargin_1;
    } else {
      iInfR->data[loop_ub] = ia->data[loop_ub];
    }
  }

  nx = bxPkOut->size[0] * bxPkOut->size[1];
  bxPkOut->size[0] = iPkOut->size[0];
  bxPkOut->size[1] = 2;
  emxEnsureCapacity_real_T(sp, bxPkOut, nx, &kc_emlrtRTEI);
  loop_ub = iPkOut->size[0] << 1;
  for (nx = 0; nx < loop_ub; nx++) {
    bxPkOut->data[nx] = 0.0;
  }

  nx = ia->size[0];
  ia->size[0] = iFinite->size[0];
  emxEnsureCapacity_int32_T(sp, ia, nx, &lc_emlrtRTEI);
  loop_ub = iFinite->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    if ((iFinite->data[nx] < 1) || (iFinite->data[nx] > iPkOut->size[0])) {
      emlrtDynamicBoundsCheckR2012b(iFinite->data[nx], 1, iPkOut->size[0],
        &nc_emlrtBCI, sp);
    }

    ia->data[nx] = iFinite->data[nx] - 1;
  }

  loop_ub = iLBw->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    if ((iLBw->data[nx] < 1) || (iLBw->data[nx] > x->size[0])) {
      emlrtDynamicBoundsCheckR2012b(iLBw->data[nx], 1, x->size[0], &oc_emlrtBCI,
        sp);
    }
  }

  b_ia[0] = ia->size[0];
  emlrtSubAssignSizeCheckR2012b(&b_ia[0], 1, &iLBw->size[0], 1, &n_emlrtECI, sp);
  loop_ub = iLBw->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    bxPkOut->data[ia->data[nx]] = x->data[iLBw->data[nx] - 1];
  }

  nx = ia->size[0];
  ia->size[0] = iFinite->size[0];
  emxEnsureCapacity_int32_T(sp, ia, nx, &mc_emlrtRTEI);
  loop_ub = iFinite->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    if ((iFinite->data[nx] < 1) || (iFinite->data[nx] > bxPkOut->size[0])) {
      emlrtDynamicBoundsCheckR2012b(iFinite->data[nx], 1, bxPkOut->size[0],
        &pc_emlrtBCI, sp);
    }

    ia->data[nx] = iFinite->data[nx] - 1;
  }

  loop_ub = iRBw->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    if ((iRBw->data[nx] < 1) || (iRBw->data[nx] > x->size[0])) {
      emlrtDynamicBoundsCheckR2012b(iRBw->data[nx], 1, x->size[0], &qc_emlrtBCI,
        sp);
    }
  }

  b_ia[0] = ia->size[0];
  emlrtSubAssignSizeCheckR2012b(&b_ia[0], 1, &iRBw->size[0], 1, &o_emlrtECI, sp);
  loop_ub = iRBw->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    bxPkOut->data[ia->data[nx] + bxPkOut->size[0]] = x->data[iRBw->data[nx] - 1];
  }

  loop_ub = iInf->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    if ((iInf->data[nx] < 1) || (iInf->data[nx] > x->size[0])) {
      emlrtDynamicBoundsCheckR2012b(iInf->data[nx], 1, x->size[0], &rc_emlrtBCI,
        sp);
    }
  }

  loop_ub = iInfL->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    if ((iInfL->data[nx] < 1) || (iInfL->data[nx] > x->size[0])) {
      emlrtDynamicBoundsCheckR2012b(iInfL->data[nx], 1, x->size[0], &sc_emlrtBCI,
        sp);
    }
  }

  if (iInf->size[0] != iInfL->size[0]) {
    emlrtSizeEqCheck1DR2012b(iInf->size[0], iInfL->size[0], &b_emlrtECI, sp);
  }

  nx = ia->size[0];
  ia->size[0] = ib->size[0];
  emxEnsureCapacity_int32_T(sp, ia, nx, &nc_emlrtRTEI);
  loop_ub = ib->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    if ((ib->data[nx] < 1) || (ib->data[nx] > bxPkOut->size[0])) {
      emlrtDynamicBoundsCheckR2012b(ib->data[nx], 1, bxPkOut->size[0],
        &tc_emlrtBCI, sp);
    }

    ia->data[nx] = ib->data[nx] - 1;
  }

  emxInit_real_T(sp, &r, 1, &fd_emlrtRTEI, true);
  nx = r->size[0];
  r->size[0] = iInf->size[0];
  emxEnsureCapacity_real_T(sp, r, nx, &oc_emlrtRTEI);
  loop_ub = iInf->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    r->data[nx] = 0.5 * (x->data[iInf->data[nx] - 1] + x->data[iInfL->data[nx] -
                         1]);
  }

  b_ia[0] = ia->size[0];
  emlrtSubAssignSizeCheckR2012b(&b_ia[0], 1, &r->size[0], 1, &p_emlrtECI, sp);
  loop_ub = r->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    bxPkOut->data[ia->data[nx]] = r->data[nx];
  }

  loop_ub = iInf->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    if ((iInf->data[nx] < 1) || (iInf->data[nx] > x->size[0])) {
      emlrtDynamicBoundsCheckR2012b(iInf->data[nx], 1, x->size[0], &uc_emlrtBCI,
        sp);
    }
  }

  loop_ub = iInfR->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    if ((iInfR->data[nx] < 1) || (iInfR->data[nx] > x->size[0])) {
      emlrtDynamicBoundsCheckR2012b(iInfR->data[nx], 1, x->size[0], &vc_emlrtBCI,
        sp);
    }
  }

  if (iInf->size[0] != iInfR->size[0]) {
    emlrtSizeEqCheck1DR2012b(iInf->size[0], iInfR->size[0], &c_emlrtECI, sp);
  }

  nx = ia->size[0];
  ia->size[0] = ib->size[0];
  emxEnsureCapacity_int32_T(sp, ia, nx, &pc_emlrtRTEI);
  loop_ub = ib->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    if ((ib->data[nx] < 1) || (ib->data[nx] > bxPkOut->size[0])) {
      emlrtDynamicBoundsCheckR2012b(ib->data[nx], 1, bxPkOut->size[0],
        &wc_emlrtBCI, sp);
    }

    ia->data[nx] = ib->data[nx] - 1;
  }

  nx = r->size[0];
  r->size[0] = iInf->size[0];
  emxEnsureCapacity_real_T(sp, r, nx, &qc_emlrtRTEI);
  loop_ub = iInf->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    r->data[nx] = 0.5 * (x->data[iInf->data[nx] - 1] + x->data[iInfR->data[nx] -
                         1]);
  }

  b_ia[0] = ia->size[0];
  emlrtSubAssignSizeCheckR2012b(&b_ia[0], 1, &r->size[0], 1, &q_emlrtECI, sp);
  loop_ub = r->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    bxPkOut->data[ia->data[nx] + bxPkOut->size[0]] = r->data[nx];
  }

  nx = byPkOut->size[0] * byPkOut->size[1];
  byPkOut->size[0] = iPkOut->size[0];
  byPkOut->size[1] = 2;
  emxEnsureCapacity_real_T(sp, byPkOut, nx, &rc_emlrtRTEI);
  loop_ub = iPkOut->size[0] << 1;
  for (nx = 0; nx < loop_ub; nx++) {
    byPkOut->data[nx] = 0.0;
  }

  nx = ia->size[0];
  ia->size[0] = iFinite->size[0];
  emxEnsureCapacity_int32_T(sp, ia, nx, &sc_emlrtRTEI);
  loop_ub = iFinite->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    if ((iFinite->data[nx] < 1) || (iFinite->data[nx] > iPkOut->size[0])) {
      emlrtDynamicBoundsCheckR2012b(iFinite->data[nx], 1, iPkOut->size[0],
        &xc_emlrtBCI, sp);
    }

    ia->data[nx] = iFinite->data[nx] - 1;
  }

  loop_ub = iLBw->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    if ((iLBw->data[nx] < 1) || (iLBw->data[nx] > y->size[0])) {
      emlrtDynamicBoundsCheckR2012b(iLBw->data[nx], 1, y->size[0], &yc_emlrtBCI,
        sp);
    }
  }

  b_ia[0] = ia->size[0];
  emlrtSubAssignSizeCheckR2012b(&b_ia[0], 1, &iLBw->size[0], 1, &f_emlrtECI, sp);
  loop_ub = iLBw->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    byPkOut->data[ia->data[nx]] = y->data[iLBw->data[nx] - 1];
  }

  nx = ia->size[0];
  ia->size[0] = iFinite->size[0];
  emxEnsureCapacity_int32_T(sp, ia, nx, &tc_emlrtRTEI);
  loop_ub = iFinite->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    if ((iFinite->data[nx] < 1) || (iFinite->data[nx] > byPkOut->size[0])) {
      emlrtDynamicBoundsCheckR2012b(iFinite->data[nx], 1, byPkOut->size[0],
        &ad_emlrtBCI, sp);
    }

    ia->data[nx] = iFinite->data[nx] - 1;
  }

  loop_ub = iRBw->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    if ((iRBw->data[nx] < 1) || (iRBw->data[nx] > y->size[0])) {
      emlrtDynamicBoundsCheckR2012b(iRBw->data[nx], 1, y->size[0], &bd_emlrtBCI,
        sp);
    }
  }

  b_ia[0] = ia->size[0];
  emlrtSubAssignSizeCheckR2012b(&b_ia[0], 1, &iRBw->size[0], 1, &g_emlrtECI, sp);
  loop_ub = iRBw->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    byPkOut->data[ia->data[nx] + byPkOut->size[0]] = y->data[iRBw->data[nx] - 1];
  }

  nx = ia->size[0];
  ia->size[0] = ib->size[0];
  emxEnsureCapacity_int32_T(sp, ia, nx, &uc_emlrtRTEI);
  loop_ub = ib->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    if ((ib->data[nx] < 1) || (ib->data[nx] > byPkOut->size[0])) {
      emlrtDynamicBoundsCheckR2012b(ib->data[nx], 1, byPkOut->size[0],
        &cd_emlrtBCI, sp);
    }

    ia->data[nx] = ib->data[nx] - 1;
  }

  loop_ub = iInfL->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    if ((iInfL->data[nx] < 1) || (iInfL->data[nx] > y->size[0])) {
      emlrtDynamicBoundsCheckR2012b(iInfL->data[nx], 1, y->size[0], &dd_emlrtBCI,
        sp);
    }
  }

  b_ia[0] = ia->size[0];
  emlrtSubAssignSizeCheckR2012b(&b_ia[0], 1, &iInfL->size[0], 1, &h_emlrtECI, sp);
  loop_ub = iInfL->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    byPkOut->data[ia->data[nx]] = y->data[iInfL->data[nx] - 1];
  }

  nx = ia->size[0];
  ia->size[0] = ib->size[0];
  emxEnsureCapacity_int32_T(sp, ia, nx, &vc_emlrtRTEI);
  loop_ub = ib->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    if ((ib->data[nx] < 1) || (ib->data[nx] > byPkOut->size[0])) {
      emlrtDynamicBoundsCheckR2012b(ib->data[nx], 1, byPkOut->size[0],
        &ed_emlrtBCI, sp);
    }

    ia->data[nx] = ib->data[nx] - 1;
  }

  loop_ub = iInfR->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    if ((iInfR->data[nx] < 1) || (iInfR->data[nx] > y->size[0])) {
      emlrtDynamicBoundsCheckR2012b(iInfR->data[nx], 1, y->size[0], &fd_emlrtBCI,
        sp);
    }
  }

  b_ia[0] = ia->size[0];
  emlrtSubAssignSizeCheckR2012b(&b_ia[0], 1, &iInfR->size[0], 1, &i_emlrtECI, sp);
  loop_ub = iInfR->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    byPkOut->data[ia->data[nx] + byPkOut->size[0]] = y->data[iInfR->data[nx] - 1];
  }

  nx = wxPkOut->size[0] * wxPkOut->size[1];
  wxPkOut->size[0] = iPkOut->size[0];
  wxPkOut->size[1] = 2;
  emxEnsureCapacity_real_T(sp, wxPkOut, nx, &wc_emlrtRTEI);
  loop_ub = iPkOut->size[0] << 1;
  for (nx = 0; nx < loop_ub; nx++) {
    wxPkOut->data[nx] = 0.0;
  }

  nx = ia->size[0];
  ia->size[0] = iFinite->size[0];
  emxEnsureCapacity_int32_T(sp, ia, nx, &xc_emlrtRTEI);
  loop_ub = iFinite->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    if ((iFinite->data[nx] < 1) || (iFinite->data[nx] > iPkOut->size[0])) {
      emlrtDynamicBoundsCheckR2012b(iFinite->data[nx], 1, iPkOut->size[0],
        &gd_emlrtBCI, sp);
    }

    ia->data[nx] = iFinite->data[nx] - 1;
  }

  emxFree_int32_T(&iFinite);
  c_ia[0] = ia->size[0];
  c_ia[1] = 2;
  emlrtSubAssignSizeCheckR2012b(&c_ia[0], 2, &wPk->size[0], 2, &j_emlrtECI, sp);
  loop_ub = wPk->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    wxPkOut->data[ia->data[nx]] = wPk->data[nx];
  }

  for (nx = 0; nx < loop_ub; nx++) {
    wxPkOut->data[ia->data[nx] + wxPkOut->size[0]] = wPk->data[nx + wPk->size[0]];
  }

  loop_ub = iInf->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    if ((iInf->data[nx] < 1) || (iInf->data[nx] > x->size[0])) {
      emlrtDynamicBoundsCheckR2012b(iInf->data[nx], 1, x->size[0], &hd_emlrtBCI,
        sp);
    }
  }

  loop_ub = iInfL->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    if ((iInfL->data[nx] < 1) || (iInfL->data[nx] > x->size[0])) {
      emlrtDynamicBoundsCheckR2012b(iInfL->data[nx], 1, x->size[0], &id_emlrtBCI,
        sp);
    }
  }

  if (iInf->size[0] != iInfL->size[0]) {
    emlrtSizeEqCheck1DR2012b(iInf->size[0], iInfL->size[0], &d_emlrtECI, sp);
  }

  nx = ia->size[0];
  ia->size[0] = ib->size[0];
  emxEnsureCapacity_int32_T(sp, ia, nx, &yc_emlrtRTEI);
  loop_ub = ib->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    if ((ib->data[nx] < 1) || (ib->data[nx] > wxPkOut->size[0])) {
      emlrtDynamicBoundsCheckR2012b(ib->data[nx], 1, wxPkOut->size[0],
        &jd_emlrtBCI, sp);
    }

    ia->data[nx] = ib->data[nx] - 1;
  }

  nx = r->size[0];
  r->size[0] = iInf->size[0];
  emxEnsureCapacity_real_T(sp, r, nx, &ad_emlrtRTEI);
  loop_ub = iInf->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    r->data[nx] = 0.5 * (x->data[iInf->data[nx] - 1] + x->data[iInfL->data[nx] -
                         1]);
  }

  emxFree_int32_T(&iInfL);
  b_ia[0] = ia->size[0];
  emlrtSubAssignSizeCheckR2012b(&b_ia[0], 1, &r->size[0], 1, &k_emlrtECI, sp);
  loop_ub = r->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    wxPkOut->data[ia->data[nx]] = r->data[nx];
  }

  loop_ub = iInf->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    if ((iInf->data[nx] < 1) || (iInf->data[nx] > x->size[0])) {
      emlrtDynamicBoundsCheckR2012b(iInf->data[nx], 1, x->size[0], &kd_emlrtBCI,
        sp);
    }
  }

  loop_ub = iInfR->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    if ((iInfR->data[nx] < 1) || (iInfR->data[nx] > x->size[0])) {
      emlrtDynamicBoundsCheckR2012b(iInfR->data[nx], 1, x->size[0], &ld_emlrtBCI,
        sp);
    }
  }

  if (iInf->size[0] != iInfR->size[0]) {
    emlrtSizeEqCheck1DR2012b(iInf->size[0], iInfR->size[0], &e_emlrtECI, sp);
  }

  nx = ia->size[0];
  ia->size[0] = ib->size[0];
  emxEnsureCapacity_int32_T(sp, ia, nx, &bd_emlrtRTEI);
  loop_ub = ib->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    if ((ib->data[nx] < 1) || (ib->data[nx] > wxPkOut->size[0])) {
      emlrtDynamicBoundsCheckR2012b(ib->data[nx], 1, wxPkOut->size[0],
        &md_emlrtBCI, sp);
    }

    ia->data[nx] = ib->data[nx] - 1;
  }

  emxFree_int32_T(&ib);
  nx = r->size[0];
  r->size[0] = iInf->size[0];
  emxEnsureCapacity_real_T(sp, r, nx, &cd_emlrtRTEI);
  loop_ub = iInf->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    r->data[nx] = 0.5 * (x->data[iInf->data[nx] - 1] + x->data[iInfR->data[nx] -
                         1]);
  }

  emxFree_int32_T(&iInfR);
  b_ia[0] = ia->size[0];
  emlrtSubAssignSizeCheckR2012b(&b_ia[0], 1, &r->size[0], 1, &l_emlrtECI, sp);
  loop_ub = r->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    wxPkOut->data[ia->data[nx] + wxPkOut->size[0]] = r->data[nx];
  }

  emxFree_int32_T(&ia);
  emxFree_real_T(&r);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

static void emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *a__output_of_sprintf_, const char_T *identifier, char_T y[23])
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = const_cast<const char *>(identifier);
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  b_emlrt_marshallIn(sp, emlrtAlias(a__output_of_sprintf_), &thisId, y);
  emlrtDestroyArray(&a__output_of_sprintf_);
}

static void findExtents(const emlrtStack *sp, const emxArray_real_T *y, const
  emxArray_real_T *x, emxArray_int32_T *iPk, const emxArray_int32_T *iFin, const
  emxArray_int32_T *iInf, const emxArray_int32_T *iInflect, emxArray_real_T *bPk,
  emxArray_real_T *bxPk, emxArray_real_T *byPk, emxArray_real_T *wxPk)
{
  emxArray_real_T *yFinite;
  int32_T i;
  int32_T loop_ub;
  emxArray_int32_T *iLeftSaddle;
  emxArray_int32_T *ii;
  emxArray_int32_T *r;
  emxArray_int32_T *r1;
  emxArray_int32_T *r2;
  emxArray_int32_T *iRightBase;
  emxArray_int32_T *iRightSaddle;
  emxArray_real_T *idx;
  emxArray_real_T *b_yFinite;
  int32_T csz_idx_0;
  emxArray_real_T *c_yFinite;
  emxArray_real_T *b_bPk;
  emxArray_boolean_T *b_x;
  int32_T b_idx;
  boolean_T exitg1;
  int32_T iv[2];
  int32_T i1;
  int32_T i2;
  emxArray_real_T *b_wxPk;
  real_T refHeight;
  real_T xc_tmp;
  real_T b_xc_tmp;
  emlrtStack st;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack h_st;
  emlrtStack i_st;
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
  f_st.prev = &e_st;
  f_st.tls = e_st.tls;
  g_st.prev = &f_st;
  g_st.tls = f_st.tls;
  h_st.prev = &g_st;
  h_st.tls = g_st.tls;
  i_st.prev = &h_st;
  i_st.tls = h_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  emxInit_real_T(sp, &yFinite, 1, &ae_emlrtRTEI, true);
  i = yFinite->size[0];
  yFinite->size[0] = y->size[0];
  emxEnsureCapacity_real_T(sp, yFinite, i, &ae_emlrtRTEI);
  loop_ub = y->size[0];
  for (i = 0; i < loop_ub; i++) {
    yFinite->data[i] = y->data[i];
  }

  loop_ub = iInf->size[0];
  for (i = 0; i < loop_ub; i++) {
    if ((iInf->data[i] < 1) || (iInf->data[i] > y->size[0])) {
      emlrtDynamicBoundsCheckR2012b(iInf->data[i], 1, y->size[0], &se_emlrtBCI,
        sp);
    }

    yFinite->data[iInf->data[i] - 1] = rtNaN;
  }

  emxInit_int32_T(sp, &iLeftSaddle, 1, &af_emlrtRTEI, true);
  emxInit_int32_T(sp, &ii, 1, &cf_emlrtRTEI, true);
  emxInit_int32_T(sp, &r, 1, &be_emlrtRTEI, true);
  st.site = &w_emlrtRSI;
  b_st.site = &bb_emlrtRSI;
  getLeftBase(&b_st, yFinite, iPk, iFin, iInflect, ii, iLeftSaddle);
  i = r->size[0];
  r->size[0] = iPk->size[0];
  emxEnsureCapacity_int32_T(&st, r, i, &be_emlrtRTEI);
  loop_ub = iPk->size[0];
  for (i = 0; i < loop_ub; i++) {
    r->data[i] = iPk->data[i];
  }

  emxInit_int32_T(&st, &r1, 1, &ce_emlrtRTEI, true);
  b_st.site = &cb_emlrtRSI;
  flipud(r);
  i = r1->size[0];
  r1->size[0] = iFin->size[0];
  emxEnsureCapacity_int32_T(&st, r1, i, &ce_emlrtRTEI);
  loop_ub = iFin->size[0];
  for (i = 0; i < loop_ub; i++) {
    r1->data[i] = iFin->data[i];
  }

  emxInit_int32_T(&st, &r2, 1, &de_emlrtRTEI, true);
  b_st.site = &cb_emlrtRSI;
  flipud(r1);
  i = r2->size[0];
  r2->size[0] = iInflect->size[0];
  emxEnsureCapacity_int32_T(&st, r2, i, &de_emlrtRTEI);
  loop_ub = iInflect->size[0];
  for (i = 0; i < loop_ub; i++) {
    r2->data[i] = iInflect->data[i];
  }

  emxInit_int32_T(&st, &iRightBase, 1, &ye_emlrtRTEI, true);
  emxInit_int32_T(&st, &iRightSaddle, 1, &bf_emlrtRTEI, true);
  b_st.site = &cb_emlrtRSI;
  flipud(r2);
  b_st.site = &cb_emlrtRSI;
  getLeftBase(&b_st, yFinite, r, r1, r2, iRightBase, iRightSaddle);
  b_st.site = &db_emlrtRSI;
  flipud(iRightBase);
  b_st.site = &eb_emlrtRSI;
  flipud(iRightSaddle);
  b_st.site = &fb_emlrtRSI;
  loop_ub = ii->size[0];
  emxFree_int32_T(&r2);
  emxFree_int32_T(&r1);
  emxFree_int32_T(&r);
  for (i = 0; i < loop_ub; i++) {
    if ((ii->data[i] < 1) || (ii->data[i] > yFinite->size[0])) {
      emlrtDynamicBoundsCheckR2012b(ii->data[i], 1, yFinite->size[0],
        &re_emlrtBCI, &b_st);
    }
  }

  loop_ub = iRightBase->size[0];
  for (i = 0; i < loop_ub; i++) {
    if ((iRightBase->data[i] < 1) || (iRightBase->data[i] > yFinite->size[0])) {
      emlrtDynamicBoundsCheckR2012b(iRightBase->data[i], 1, yFinite->size[0],
        &qe_emlrtBCI, &b_st);
    }
  }

  emxInit_real_T(&b_st, &idx, 1, &ke_emlrtRTEI, true);
  emxInit_real_T(&b_st, &b_yFinite, 1, &fe_emlrtRTEI, true);
  c_st.site = &hb_emlrtRSI;
  d_st.site = &ib_emlrtRSI;
  e_st.site = &jb_emlrtRSI;
  f_st.site = &kb_emlrtRSI;
  g_st.site = &lb_emlrtRSI;
  if (ii->size[0] <= iRightBase->size[0]) {
    csz_idx_0 = ii->size[0];
  } else {
    csz_idx_0 = iRightBase->size[0];
  }

  i = idx->size[0];
  if (ii->size[0] <= iRightBase->size[0]) {
    idx->size[0] = ii->size[0];
  } else {
    idx->size[0] = iRightBase->size[0];
  }

  emxEnsureCapacity_real_T(&g_st, idx, i, &ee_emlrtRTEI);
  i = b_yFinite->size[0];
  b_yFinite->size[0] = ii->size[0];
  emxEnsureCapacity_real_T(&g_st, b_yFinite, i, &fe_emlrtRTEI);
  loop_ub = ii->size[0];
  for (i = 0; i < loop_ub; i++) {
    b_yFinite->data[i] = yFinite->data[ii->data[i] - 1];
  }

  emxInit_real_T(&g_st, &c_yFinite, 1, &ge_emlrtRTEI, true);
  i = c_yFinite->size[0];
  c_yFinite->size[0] = iRightBase->size[0];
  emxEnsureCapacity_real_T(&g_st, c_yFinite, i, &ge_emlrtRTEI);
  loop_ub = iRightBase->size[0];
  for (i = 0; i < loop_ub; i++) {
    c_yFinite->data[i] = yFinite->data[iRightBase->data[i] - 1];
  }

  if (!dimagree(idx, b_yFinite, c_yFinite)) {
    emlrtErrorWithMessageIdR2018a(&g_st, &p_emlrtRTEI, "MATLAB:dimagree",
      "MATLAB:dimagree", 0);
  }

  emxFree_real_T(&c_yFinite);
  emxFree_real_T(&b_yFinite);
  emxInit_real_T(&g_st, &b_bPk, 1, &we_emlrtRTEI, true);
  i = b_bPk->size[0];
  b_bPk->size[0] = csz_idx_0;
  emxEnsureCapacity_real_T(&f_st, b_bPk, i, &ic_emlrtRTEI);
  g_st.site = &mb_emlrtRSI;
  h_st.site = &nb_emlrtRSI;
  if ((1 <= idx->size[0]) && (idx->size[0] > 2147483646)) {
    i_st.site = &u_emlrtRSI;
    check_forloop_overflow_error(&i_st);
  }

  for (loop_ub = 0; loop_ub < csz_idx_0; loop_ub++) {
    b_bPk->data[loop_ub] = muDoubleScalarMax(yFinite->data[ii->data[loop_ub] - 1],
      yFinite->data[iRightBase->data[loop_ub] - 1]);
  }

  emxFree_int32_T(&iRightBase);
  st.site = &x_emlrtRSI;
  loop_ub = iPk->size[0];
  for (i = 0; i < loop_ub; i++) {
    if ((iPk->data[i] < 1) || (iPk->data[i] > yFinite->size[0])) {
      emlrtDynamicBoundsCheckR2012b(iPk->data[i], 1, yFinite->size[0],
        &pe_emlrtBCI, &st);
    }
  }

  i = iPk->size[0];
  if (i != b_bPk->size[0]) {
    emlrtSizeEqCheck1DR2012b(i, b_bPk->size[0], &t_emlrtECI, &st);
  }

  emxInit_boolean_T(&st, &b_x, 1, &he_emlrtRTEI, true);
  b_st.site = &ob_emlrtRSI;
  i = b_x->size[0];
  b_x->size[0] = iPk->size[0];
  emxEnsureCapacity_boolean_T(&b_st, b_x, i, &he_emlrtRTEI);
  loop_ub = iPk->size[0];
  for (i = 0; i < loop_ub; i++) {
    b_x->data[i] = (yFinite->data[iPk->data[i] - 1] - b_bPk->data[i] >= 8.0);
  }

  c_st.site = &pb_emlrtRSI;
  csz_idx_0 = b_x->size[0];
  d_st.site = &qb_emlrtRSI;
  b_idx = 0;
  i = ii->size[0];
  ii->size[0] = b_x->size[0];
  emxEnsureCapacity_int32_T(&d_st, ii, i, &ie_emlrtRTEI);
  e_st.site = &rb_emlrtRSI;
  if ((1 <= b_x->size[0]) && (b_x->size[0] > 2147483646)) {
    f_st.site = &u_emlrtRSI;
    check_forloop_overflow_error(&f_st);
  }

  loop_ub = 0;
  exitg1 = false;
  while ((!exitg1) && (loop_ub <= csz_idx_0 - 1)) {
    if (b_x->data[loop_ub]) {
      b_idx++;
      ii->data[b_idx - 1] = loop_ub + 1;
      if (b_idx >= csz_idx_0) {
        exitg1 = true;
      } else {
        loop_ub++;
      }
    } else {
      loop_ub++;
    }
  }

  if (b_idx > b_x->size[0]) {
    emlrtErrorWithMessageIdR2018a(&d_st, &q_emlrtRTEI,
      "Coder:builtins:AssertionFailed", "Coder:builtins:AssertionFailed", 0);
  }

  if (b_x->size[0] == 1) {
    if (b_idx == 0) {
      ii->size[0] = 0;
    }
  } else {
    if (1 > b_idx) {
      i = 0;
    } else {
      i = b_idx;
    }

    iv[0] = 1;
    iv[1] = i;
    e_st.site = &sb_emlrtRSI;
    indexShapeCheck(&e_st, ii->size[0], iv);
    i1 = ii->size[0];
    ii->size[0] = i;
    emxEnsureCapacity_int32_T(&d_st, ii, i1, &je_emlrtRTEI);
  }

  emxFree_boolean_T(&b_x);
  i = idx->size[0];
  idx->size[0] = ii->size[0];
  emxEnsureCapacity_real_T(&b_st, idx, i, &ke_emlrtRTEI);
  loop_ub = ii->size[0];
  for (i = 0; i < loop_ub; i++) {
    idx->data[i] = ii->data[i];
  }

  i = iPk->size[0];
  i1 = ii->size[0];
  ii->size[0] = idx->size[0];
  emxEnsureCapacity_int32_T(&st, ii, i1, &le_emlrtRTEI);
  loop_ub = idx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    i2 = static_cast<int32_T>(idx->data[i1]);
    if ((i2 < 1) || (i2 > i)) {
      emlrtDynamicBoundsCheckR2012b(i2, 1, i, &oe_emlrtBCI, &st);
    }

    ii->data[i1] = iPk->data[i2 - 1];
  }

  i = iPk->size[0];
  iPk->size[0] = ii->size[0];
  emxEnsureCapacity_int32_T(&st, iPk, i, &me_emlrtRTEI);
  loop_ub = ii->size[0];
  for (i = 0; i < loop_ub; i++) {
    iPk->data[i] = ii->data[i];
  }

  i = bPk->size[0];
  bPk->size[0] = idx->size[0];
  emxEnsureCapacity_real_T(&st, bPk, i, &ne_emlrtRTEI);
  loop_ub = idx->size[0];
  for (i = 0; i < loop_ub; i++) {
    i1 = static_cast<int32_T>(idx->data[i]);
    if ((i1 < 1) || (i1 > b_bPk->size[0])) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, b_bPk->size[0], &ne_emlrtBCI, &st);
    }

    bPk->data[i] = b_bPk->data[i1 - 1];
  }

  i = b_bPk->size[0];
  b_bPk->size[0] = bPk->size[0];
  emxEnsureCapacity_real_T(&st, b_bPk, i, &oe_emlrtRTEI);
  loop_ub = bPk->size[0];
  for (i = 0; i < loop_ub; i++) {
    b_bPk->data[i] = bPk->data[i];
  }

  i = ii->size[0];
  ii->size[0] = idx->size[0];
  emxEnsureCapacity_int32_T(&st, ii, i, &pe_emlrtRTEI);
  loop_ub = idx->size[0];
  for (i = 0; i < loop_ub; i++) {
    i1 = static_cast<int32_T>(idx->data[i]);
    if ((i1 < 1) || (i1 > iLeftSaddle->size[0])) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, iLeftSaddle->size[0], &me_emlrtBCI,
        &st);
    }

    ii->data[i] = iLeftSaddle->data[i1 - 1];
  }

  i = iLeftSaddle->size[0];
  iLeftSaddle->size[0] = ii->size[0];
  emxEnsureCapacity_int32_T(&st, iLeftSaddle, i, &qe_emlrtRTEI);
  loop_ub = ii->size[0];
  for (i = 0; i < loop_ub; i++) {
    iLeftSaddle->data[i] = ii->data[i];
  }

  i = ii->size[0];
  ii->size[0] = idx->size[0];
  emxEnsureCapacity_int32_T(&st, ii, i, &re_emlrtRTEI);
  loop_ub = idx->size[0];
  for (i = 0; i < loop_ub; i++) {
    i1 = static_cast<int32_T>(idx->data[i]);
    if ((i1 < 1) || (i1 > iRightSaddle->size[0])) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, iRightSaddle->size[0], &le_emlrtBCI,
        &st);
    }

    ii->data[i] = iRightSaddle->data[i1 - 1];
  }

  i = iRightSaddle->size[0];
  iRightSaddle->size[0] = ii->size[0];
  emxEnsureCapacity_int32_T(&st, iRightSaddle, i, &se_emlrtRTEI);
  loop_ub = ii->size[0];
  for (i = 0; i < loop_ub; i++) {
    iRightSaddle->data[i] = ii->data[i];
  }

  st.site = &y_emlrtRSI;
  if (iPk->size[0] == 0) {
    idx->size[0] = 0;
    iLeftSaddle->size[0] = 0;
    iRightSaddle->size[0] = 0;
  } else {
    i = idx->size[0];
    idx->size[0] = b_bPk->size[0];
    emxEnsureCapacity_real_T(&st, idx, i, &te_emlrtRTEI);
    loop_ub = b_bPk->size[0];
    for (i = 0; i < loop_ub; i++) {
      idx->data[i] = b_bPk->data[i];
    }
  }

  emxInit_real_T(&st, &b_wxPk, 2, &xe_emlrtRTEI, true);
  b_st.site = &ub_emlrtRSI;
  i = b_wxPk->size[0] * b_wxPk->size[1];
  b_wxPk->size[0] = iPk->size[0];
  b_wxPk->size[1] = 2;
  emxEnsureCapacity_real_T(&b_st, b_wxPk, i, &ue_emlrtRTEI);
  loop_ub = iPk->size[0] << 1;
  for (i = 0; i < loop_ub; i++) {
    b_wxPk->data[i] = 0.0;
  }

  i = iPk->size[0];
  for (b_idx = 0; b_idx < i; b_idx++) {
    i1 = static_cast<int32_T>((b_idx + 1U));
    if ((i1 < 1) || (i1 > idx->size[0])) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, idx->size[0], &je_emlrtBCI, &b_st);
    }

    i2 = iPk->size[0];
    if (i1 > i2) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, i2, &ke_emlrtBCI, &b_st);
    }

    i2 = iPk->data[i1 - 1];
    if ((i2 < 1) || (i2 > yFinite->size[0])) {
      emlrtDynamicBoundsCheckR2012b(iPk->data[i1 - 1], 1, yFinite->size[0],
        &ke_emlrtBCI, &b_st);
    }

    refHeight = (yFinite->data[i2 - 1] + idx->data[i1 - 1]) / 2.0;
    c_st.site = &vb_emlrtRSI;
    i2 = iPk->size[0];
    if (i1 > i2) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, i2, &ie_emlrtBCI, &c_st);
    }

    csz_idx_0 = iPk->data[i1 - 1];
    i2 = b_idx + 1;
    if ((i2 < 1) || (i2 > iLeftSaddle->size[0])) {
      emlrtDynamicBoundsCheckR2012b(i2, 1, iLeftSaddle->size[0], &te_emlrtBCI,
        &c_st);
    }

    exitg1 = false;
    while ((!exitg1) && (csz_idx_0 >= iLeftSaddle->data[b_idx])) {
      if ((csz_idx_0 < 1) || (csz_idx_0 > yFinite->size[0])) {
        emlrtDynamicBoundsCheckR2012b(csz_idx_0, 1, yFinite->size[0],
          &he_emlrtBCI, &c_st);
      }

      if (yFinite->data[csz_idx_0 - 1] > refHeight) {
        csz_idx_0--;
      } else {
        exitg1 = true;
      }
    }

    if (i1 > iLeftSaddle->size[0]) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, iLeftSaddle->size[0], &ge_emlrtBCI,
        &b_st);
    }

    if (csz_idx_0 < iLeftSaddle->data[i1 - 1]) {
      if (i1 > b_wxPk->size[0]) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, b_wxPk->size[0], &ee_emlrtBCI,
          &b_st);
      }

      if (i1 > iLeftSaddle->size[0]) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, iLeftSaddle->size[0], &fe_emlrtBCI,
          &b_st);
      }

      i2 = iLeftSaddle->data[i1 - 1];
      if ((i2 < 1) || (i2 > x->size[0])) {
        emlrtDynamicBoundsCheckR2012b(iLeftSaddle->data[i1 - 1], 1, x->size[0],
          &fe_emlrtBCI, &b_st);
      }

      b_wxPk->data[i1 - 1] = x->data[i2 - 1];
    } else {
      if ((csz_idx_0 < 1) || (csz_idx_0 > x->size[0])) {
        emlrtDynamicBoundsCheckR2012b(csz_idx_0, 1, x->size[0], &ue_emlrtBCI,
          &b_st);
      }

      i2 = csz_idx_0 + 1;
      if ((i2 < 1) || (i2 > x->size[0])) {
        emlrtDynamicBoundsCheckR2012b(i2, 1, x->size[0], &ve_emlrtBCI, &b_st);
      }

      if (csz_idx_0 > yFinite->size[0]) {
        emlrtDynamicBoundsCheckR2012b(csz_idx_0, 1, yFinite->size[0],
          &we_emlrtBCI, &b_st);
      }

      i2 = csz_idx_0 + 1;
      if ((i2 < 1) || (i2 > yFinite->size[0])) {
        emlrtDynamicBoundsCheckR2012b(i2, 1, yFinite->size[0], &xe_emlrtBCI,
          &b_st);
      }

      i2 = iPk->size[0];
      if (i1 > i2) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, i2, &ce_emlrtBCI, &b_st);
      }

      i2 = iPk->data[i1 - 1];
      if ((i2 < 1) || (i2 > yFinite->size[0])) {
        emlrtDynamicBoundsCheckR2012b(iPk->data[i1 - 1], 1, yFinite->size[0],
          &ce_emlrtBCI, &b_st);
      }

      i2 = b_idx + 1;
      if ((i2 < 1) || (i2 > idx->size[0])) {
        emlrtDynamicBoundsCheckR2012b(i2, 1, idx->size[0], &ye_emlrtBCI, &b_st);
      }

      xc_tmp = yFinite->data[csz_idx_0 - 1];
      b_xc_tmp = x->data[csz_idx_0 - 1];
      xc_tmp = b_xc_tmp + (x->data[csz_idx_0] - b_xc_tmp) * (0.5 *
        (yFinite->data[iPk->data[b_idx] - 1] + idx->data[b_idx]) - xc_tmp) /
        (yFinite->data[csz_idx_0] - xc_tmp);
      if (muDoubleScalarIsNaN(xc_tmp)) {
        xc_tmp = x->data[csz_idx_0];
      }

      if (i1 > b_wxPk->size[0]) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, b_wxPk->size[0], &ae_emlrtBCI,
          &b_st);
      }

      b_wxPk->data[i1 - 1] = xc_tmp;
    }

    c_st.site = &wb_emlrtRSI;
    i2 = iPk->size[0];
    if (i1 > i2) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, i2, &de_emlrtBCI, &c_st);
    }

    csz_idx_0 = iPk->data[i1 - 1];
    i2 = b_idx + 1;
    if ((i2 < 1) || (i2 > iRightSaddle->size[0])) {
      emlrtDynamicBoundsCheckR2012b(i2, 1, iRightSaddle->size[0], &af_emlrtBCI,
        &c_st);
    }

    exitg1 = false;
    while ((!exitg1) && (csz_idx_0 <= iRightSaddle->data[b_idx])) {
      if ((csz_idx_0 < 1) || (csz_idx_0 > yFinite->size[0])) {
        emlrtDynamicBoundsCheckR2012b(csz_idx_0, 1, yFinite->size[0],
          &be_emlrtBCI, &c_st);
      }

      if (yFinite->data[csz_idx_0 - 1] > refHeight) {
        csz_idx_0++;
      } else {
        exitg1 = true;
      }
    }

    if (i1 > iRightSaddle->size[0]) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, iRightSaddle->size[0], &yd_emlrtBCI,
        &b_st);
    }

    if (csz_idx_0 > iRightSaddle->data[i1 - 1]) {
      if (i1 > b_wxPk->size[0]) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, b_wxPk->size[0], &wd_emlrtBCI,
          &b_st);
      }

      if (i1 > iRightSaddle->size[0]) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, iRightSaddle->size[0], &xd_emlrtBCI,
          &b_st);
      }

      i2 = iRightSaddle->data[i1 - 1];
      if ((i2 < 1) || (i2 > x->size[0])) {
        emlrtDynamicBoundsCheckR2012b(iRightSaddle->data[i1 - 1], 1, x->size[0],
          &xd_emlrtBCI, &b_st);
      }

      b_wxPk->data[(i1 + b_wxPk->size[0]) - 1] = x->data[i2 - 1];
    } else {
      if ((csz_idx_0 < 1) || (csz_idx_0 > x->size[0])) {
        emlrtDynamicBoundsCheckR2012b(csz_idx_0, 1, x->size[0], &bf_emlrtBCI,
          &b_st);
      }

      i2 = csz_idx_0 - 1;
      if ((i2 < 1) || (i2 > x->size[0])) {
        emlrtDynamicBoundsCheckR2012b(i2, 1, x->size[0], &cf_emlrtBCI, &b_st);
      }

      if (csz_idx_0 > yFinite->size[0]) {
        emlrtDynamicBoundsCheckR2012b(csz_idx_0, 1, yFinite->size[0],
          &df_emlrtBCI, &b_st);
      }

      i2 = csz_idx_0 - 1;
      if ((i2 < 1) || (i2 > yFinite->size[0])) {
        emlrtDynamicBoundsCheckR2012b(i2, 1, yFinite->size[0], &ef_emlrtBCI,
          &b_st);
      }

      i2 = iPk->size[0];
      if (i1 > i2) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, i2, &vd_emlrtBCI, &b_st);
      }

      i2 = iPk->data[i1 - 1];
      if ((i2 < 1) || (i2 > yFinite->size[0])) {
        emlrtDynamicBoundsCheckR2012b(iPk->data[i1 - 1], 1, yFinite->size[0],
          &vd_emlrtBCI, &b_st);
      }

      i2 = b_idx + 1;
      if ((i2 < 1) || (i2 > idx->size[0])) {
        emlrtDynamicBoundsCheckR2012b(i2, 1, idx->size[0], &ff_emlrtBCI, &b_st);
      }

      xc_tmp = yFinite->data[csz_idx_0 - 1];
      b_xc_tmp = x->data[csz_idx_0 - 1];
      refHeight = x->data[csz_idx_0 - 2];
      xc_tmp = b_xc_tmp + (refHeight - b_xc_tmp) * (0.5 * (yFinite->data
        [iPk->data[b_idx] - 1] + idx->data[b_idx]) - xc_tmp) / (yFinite->
        data[csz_idx_0 - 2] - xc_tmp);
      if (muDoubleScalarIsNaN(xc_tmp)) {
        xc_tmp = refHeight;
      }

      if (i1 > b_wxPk->size[0]) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, b_wxPk->size[0], &ud_emlrtBCI,
          &b_st);
      }

      b_wxPk->data[(i1 + b_wxPk->size[0]) - 1] = xc_tmp;
    }
  }

  emxFree_real_T(&idx);
  emxFree_real_T(&yFinite);
  i = ii->size[0];
  ii->size[0] = iPk->size[0];
  emxEnsureCapacity_int32_T(sp, ii, i, &ve_emlrtRTEI);
  loop_ub = iPk->size[0] - 1;
  for (i = 0; i <= loop_ub; i++) {
    ii->data[i] = iPk->data[i];
  }

  st.site = &ab_emlrtRSI;
  combineFullPeaks(&st, y, x, ii, b_bPk, iLeftSaddle, iRightSaddle, b_wxPk, iInf,
                   iPk, bPk, bxPk, byPk, wxPk);
  emxFree_int32_T(&ii);
  emxFree_int32_T(&iRightSaddle);
  emxFree_int32_T(&iLeftSaddle);
  emxFree_real_T(&b_wxPk);
  emxFree_real_T(&b_bPk);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, char_T ret[23])
{
  static const int32_T dims[2] = { 1, 23 };

  emlrtCheckBuiltInR2012b(sp, msgId, src, "char", false, 2U, dims);
  emlrtImportCharArrayR2015b(sp, src, &ret[0], 23);
  emlrtDestroyArray(&src);
}

static void getLeftBase(const emlrtStack *sp, const emxArray_real_T *yTemp,
  const emxArray_int32_T *iPeak, const emxArray_int32_T *iFinite, const
  emxArray_int32_T *iInflect, emxArray_int32_T *iBase, emxArray_int32_T *iSaddle)
{
  int32_T i;
  int32_T n;
  emxArray_real_T *peak;
  emxArray_real_T *valley;
  emxArray_int32_T *iValley;
  int32_T b_i;
  int32_T j;
  int32_T k;
  real_T v;
  int32_T iv;
  int32_T exitg1;
  real_T p;
  boolean_T exitg2;
  int32_T isv;
  real_T d;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  i = iBase->size[0];
  iBase->size[0] = iPeak->size[0];
  emxEnsureCapacity_int32_T(sp, iBase, i, &xb_emlrtRTEI);
  n = iPeak->size[0];
  for (i = 0; i < n; i++) {
    iBase->data[i] = 0;
  }

  i = iSaddle->size[0];
  iSaddle->size[0] = iPeak->size[0];
  emxEnsureCapacity_int32_T(sp, iSaddle, i, &yb_emlrtRTEI);
  n = iPeak->size[0];
  for (i = 0; i < n; i++) {
    iSaddle->data[i] = 0;
  }

  emxInit_real_T(sp, &peak, 1, &ac_emlrtRTEI, true);
  i = peak->size[0];
  peak->size[0] = iFinite->size[0];
  emxEnsureCapacity_real_T(sp, peak, i, &ac_emlrtRTEI);
  n = iFinite->size[0];
  for (i = 0; i < n; i++) {
    peak->data[i] = 0.0;
  }

  emxInit_real_T(sp, &valley, 1, &bc_emlrtRTEI, true);
  i = valley->size[0];
  valley->size[0] = iFinite->size[0];
  emxEnsureCapacity_real_T(sp, valley, i, &bc_emlrtRTEI);
  n = iFinite->size[0];
  for (i = 0; i < n; i++) {
    valley->data[i] = 0.0;
  }

  emxInit_int32_T(sp, &iValley, 1, &cc_emlrtRTEI, true);
  i = iValley->size[0];
  iValley->size[0] = iFinite->size[0];
  emxEnsureCapacity_int32_T(sp, iValley, i, &cc_emlrtRTEI);
  n = iFinite->size[0];
  for (i = 0; i < n; i++) {
    iValley->data[i] = 0;
  }

  n = 0;
  b_i = 1;
  j = 1;
  k = 1;
  v = rtNaN;
  iv = 1;
  while (k <= iPeak->size[0]) {
    do {
      exitg1 = 0;
      if ((b_i < 1) || (b_i > iInflect->size[0])) {
        emlrtDynamicBoundsCheckR2012b(b_i, 1, iInflect->size[0], &pb_emlrtBCI,
          sp);
      }

      if ((j < 1) || (j > iFinite->size[0])) {
        emlrtDynamicBoundsCheckR2012b(j, 1, iFinite->size[0], &qb_emlrtBCI, sp);
      }

      i = iInflect->data[b_i - 1];
      if (i != iFinite->data[j - 1]) {
        if (b_i > iInflect->size[0]) {
          emlrtDynamicBoundsCheckR2012b(b_i, 1, iInflect->size[0], &sb_emlrtBCI,
            sp);
        }

        if ((i < 1) || (i > yTemp->size[0])) {
          emlrtDynamicBoundsCheckR2012b(i, 1, yTemp->size[0], &sb_emlrtBCI, sp);
        }

        v = yTemp->data[i - 1];
        if (b_i > iInflect->size[0]) {
          emlrtDynamicBoundsCheckR2012b(b_i, 1, iInflect->size[0], &tb_emlrtBCI,
            sp);
        }

        iv = i;
        if (muDoubleScalarIsNaN(yTemp->data[iInflect->data[b_i - 1] - 1])) {
          n = 0;
        } else {
          exitg2 = false;
          while ((!exitg2) && (n > 0)) {
            if (n > valley->size[0]) {
              emlrtDynamicBoundsCheckR2012b(n, 1, valley->size[0], &xb_emlrtBCI,
                sp);
            }

            if (valley->data[n - 1] > v) {
              n--;
            } else {
              exitg2 = true;
            }
          }
        }

        b_i++;
      } else {
        exitg1 = 1;
      }
    } while (exitg1 == 0);

    if (b_i > iInflect->size[0]) {
      emlrtDynamicBoundsCheckR2012b(b_i, 1, iInflect->size[0], &rb_emlrtBCI, sp);
    }

    if ((i < 1) || (i > yTemp->size[0])) {
      emlrtDynamicBoundsCheckR2012b(i, 1, yTemp->size[0], &rb_emlrtBCI, sp);
    }

    p = yTemp->data[i - 1];
    exitg2 = false;
    while ((!exitg2) && (n > 0)) {
      if (n > peak->size[0]) {
        emlrtDynamicBoundsCheckR2012b(n, 1, peak->size[0], &ub_emlrtBCI, sp);
      }

      if (peak->data[n - 1] < p) {
        if (n > valley->size[0]) {
          emlrtDynamicBoundsCheckR2012b(n, 1, valley->size[0], &vb_emlrtBCI, sp);
        }

        d = valley->data[n - 1];
        if (d < v) {
          if (n > valley->size[0]) {
            emlrtDynamicBoundsCheckR2012b(n, 1, valley->size[0], &wb_emlrtBCI,
              sp);
          }

          v = d;
          if (n > iValley->size[0]) {
            emlrtDynamicBoundsCheckR2012b(n, 1, iValley->size[0], &yb_emlrtBCI,
              sp);
          }

          iv = iValley->data[n - 1];
        }

        n--;
      } else {
        exitg2 = true;
      }
    }

    isv = iv;
    exitg2 = false;
    while ((!exitg2) && (n > 0)) {
      if (n > peak->size[0]) {
        emlrtDynamicBoundsCheckR2012b(n, 1, peak->size[0], &ac_emlrtBCI, sp);
      }

      if (peak->data[n - 1] <= p) {
        if (n > valley->size[0]) {
          emlrtDynamicBoundsCheckR2012b(n, 1, valley->size[0], &bc_emlrtBCI, sp);
        }

        d = valley->data[n - 1];
        if (d < v) {
          if (n > valley->size[0]) {
            emlrtDynamicBoundsCheckR2012b(n, 1, valley->size[0], &dc_emlrtBCI,
              sp);
          }

          v = d;
          if (n > iValley->size[0]) {
            emlrtDynamicBoundsCheckR2012b(n, 1, iValley->size[0], &fc_emlrtBCI,
              sp);
          }

          iv = iValley->data[n - 1];
        }

        n--;
      } else {
        exitg2 = true;
      }
    }

    n++;
    if ((n < 1) || (n > peak->size[0])) {
      emlrtDynamicBoundsCheckR2012b(n, 1, peak->size[0], &cc_emlrtBCI, sp);
    }

    peak->data[n - 1] = yTemp->data[iInflect->data[b_i - 1] - 1];
    if (n > valley->size[0]) {
      emlrtDynamicBoundsCheckR2012b(n, 1, valley->size[0], &ec_emlrtBCI, sp);
    }

    valley->data[n - 1] = v;
    if (n > iValley->size[0]) {
      emlrtDynamicBoundsCheckR2012b(n, 1, iValley->size[0], &gc_emlrtBCI, sp);
    }

    iValley->data[n - 1] = iv;
    if (b_i > iInflect->size[0]) {
      emlrtDynamicBoundsCheckR2012b(b_i, 1, iInflect->size[0], &hc_emlrtBCI, sp);
    }

    if ((k < 1) || (k > iPeak->size[0])) {
      emlrtDynamicBoundsCheckR2012b(k, 1, iPeak->size[0], &ic_emlrtBCI, sp);
    }

    if (i == iPeak->data[k - 1]) {
      if (k > iBase->size[0]) {
        emlrtDynamicBoundsCheckR2012b(k, 1, iBase->size[0], &jc_emlrtBCI, sp);
      }

      iBase->data[k - 1] = iv;
      if (k > iSaddle->size[0]) {
        emlrtDynamicBoundsCheckR2012b(k, 1, iSaddle->size[0], &kc_emlrtBCI, sp);
      }

      iSaddle->data[k - 1] = isv;
      k++;
    }

    b_i++;
    j++;
  }

  emxFree_int32_T(&iValley);
  emxFree_real_T(&valley);
  emxFree_real_T(&peak);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

static void parse_inputs(const emlrtStack *sp, const emxArray_real_T *Yin, const
  emxArray_real_T *varargin_1, emxArray_real_T *y, emxArray_real_T *x, real_T
  *NpOut)
{
  int32_T k;
  int32_T loop_ub;
  boolean_T p;
  boolean_T exitg1;
  real_T attributes_f6;
  const mxArray *b_y;
  const mxArray *m;
  static const int32_T iv[2] = { 1, 7 };

  static const char_T rfmt[7] = { '%', '2', '3', '.', '1', '5', 'e' };

  const mxArray *c_y;
  const mxArray *m1;
  char_T numstr[23];
  emlrtStack st;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &l_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  f_st.prev = &e_st;
  f_st.tls = e_st.tls;
  b_st.site = &p_emlrtRSI;
  if (Yin->size[1] == 0) {
    emlrtErrorWithMessageIdR2018a(&b_st, &d_emlrtRTEI,
      "Coder:toolbox:ValidateattributesexpectedNonempty",
      "MATLAB:findpeaks:expectedNonempty", 3, 4, 1, "Y");
  }

  k = y->size[0];
  y->size[0] = Yin->size[1];
  emxEnsureCapacity_real_T(sp, y, k, &vb_emlrtRTEI);
  loop_ub = Yin->size[1];
  for (k = 0; k < loop_ub; k++) {
    y->data[k] = Yin->data[k];
  }

  if (Yin->size[1] < 3) {
    emlrtErrorWithMessageIdR2018a(sp, &b_emlrtRTEI,
      "signal:findpeaks:emptyDataSet", "signal:findpeaks:emptyDataSet", 0);
  }

  st.site = &m_emlrtRSI;
  b_st.site = &p_emlrtRSI;
  p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= varargin_1->size[1] - 1)) {
    if (!muDoubleScalarIsNaN(varargin_1->data[k])) {
      k++;
    } else {
      p = false;
      exitg1 = true;
    }
  }

  if (!p) {
    emlrtErrorWithMessageIdR2018a(&b_st, &e_emlrtRTEI,
      "Coder:toolbox:ValidateattributesexpectedFinite",
      "MATLAB:findpeaks:expectedFinite", 3, 4, 1, "X");
  }

  b_st.site = &p_emlrtRSI;
  if (varargin_1->size[1] > 1) {
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k <= varargin_1->size[1] - 2)) {
      if (!(varargin_1->data[k] < varargin_1->data[k + 1])) {
        emlrtErrorWithMessageIdR2018a(&b_st, &f_emlrtRTEI,
          "Coder:toolbox:ValidateattributesexpectedIncreasing",
          "MATLAB:findpeaks:expectedIncreasing", 3, 4, 1, "X");
      } else {
        k++;
      }
    }
  }

  if (varargin_1->size[1] != Yin->size[1]) {
    emlrtErrorWithMessageIdR2018a(sp, &c_emlrtRTEI,
      "signal:findpeaks:mismatchYX", "signal:findpeaks:mismatchYX", 0);
  }

  k = x->size[0];
  x->size[0] = varargin_1->size[1];
  emxEnsureCapacity_real_T(sp, x, k, &wb_emlrtRTEI);
  loop_ub = varargin_1->size[1];
  for (k = 0; k < loop_ub; k++) {
    x->data[k] = varargin_1->data[k];
  }

  k = Yin->size[1];
  if ((Yin->size[1] < 1) || (Yin->size[1] > varargin_1->size[1])) {
    emlrtDynamicBoundsCheckR2012b(Yin->size[1], 1, varargin_1->size[1],
      &ob_emlrtBCI, sp);
  }

  attributes_f6 = varargin_1->data[Yin->size[1] - 1] - varargin_1->data[0];
  st.site = &n_emlrtRSI;
  b_st.site = &p_emlrtRSI;
  p = (50.0 < attributes_f6);
  if (!p) {
    c_st.site = &q_emlrtRSI;
    d_st.site = &r_emlrtRSI;
    e_st.site = &s_emlrtRSI;
    b_y = NULL;
    m = emlrtCreateCharArray(2, iv);
    emlrtInitCharArrayR2013a(&e_st, 7, m, &rfmt[0]);
    emlrtAssign(&b_y, m);
    c_y = NULL;
    m1 = emlrtCreateDoubleScalar(attributes_f6);
    emlrtAssign(&c_y, m1);
    f_st.site = &me_emlrtRSI;
    emlrt_marshallIn(&f_st, b_sprintf(&f_st, b_y, c_y, &emlrtMCI),
                     "<output of sprintf>", numstr);
    emlrtErrorWithMessageIdR2018a(&b_st, &g_emlrtRTEI,
      "MATLAB:validateattributes:expectedScalar", "MATLAB:findpeaks:notLess", 9,
      4, 15, "MinPeakDistance", 4, 1, "<", 4, 23, numstr);
  }

  st.site = &o_emlrtRSI;
  b_st.site = &p_emlrtRSI;
  *NpOut = k;
}

void findpeaks(const emlrtStack *sp, const emxArray_real_T *Yin, const
               emxArray_real_T *varargin_1, emxArray_real_T *Ypk,
               emxArray_real_T *Xpk)
{
  emxArray_int32_T *idx;
  emxArray_real_T *y;
  emxArray_real_T *x;
  emxArray_int32_T *iInfinite;
  emxArray_int32_T *iInflect;
  real_T Np;
  int32_T kfirst;
  int32_T ny;
  int32_T nPk;
  int32_T nInflect;
  char_T dir;
  real_T ykfirst;
  boolean_T isinfykfirst;
  int32_T k;
  boolean_T guard1 = false;
  real_T yk;
  boolean_T isinfyk;
  char_T previousdir;
  emxArray_int32_T *iPk;
  emxArray_int32_T *b_iPk;
  emxArray_real_T *bPk;
  emxArray_real_T *bxPk;
  emxArray_real_T *byPk;
  emxArray_real_T *wxPk;
  int32_T iv[2];
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
  emxInit_int32_T(sp, &idx, 1, &nb_emlrtRTEI, true);
  emxInit_real_T(sp, &y, 1, &ob_emlrtRTEI, true);
  emxInit_real_T(sp, &x, 1, &pb_emlrtRTEI, true);
  emxInit_int32_T(sp, &iInfinite, 1, &gb_emlrtRTEI, true);
  emxInit_int32_T(sp, &iInflect, 1, &qb_emlrtRTEI, true);
  st.site = &d_emlrtRSI;
  parse_inputs(&st, Yin, varargin_1, y, x, &Np);
  st.site = &e_emlrtRSI;
  kfirst = idx->size[0];
  idx->size[0] = y->size[0];
  emxEnsureCapacity_int32_T(&st, idx, kfirst, &y_emlrtRTEI);
  kfirst = iInflect->size[0];
  iInflect->size[0] = y->size[0];
  emxEnsureCapacity_int32_T(&st, iInflect, kfirst, &y_emlrtRTEI);
  ny = y->size[0];
  nPk = 0;
  nInflect = 0;
  dir = 'n';
  kfirst = 0;
  ykfirst = rtInf;
  isinfykfirst = true;
  b_st.site = &t_emlrtRSI;
  if ((1 <= y->size[0]) && (y->size[0] > 2147483646)) {
    c_st.site = &u_emlrtRSI;
    check_forloop_overflow_error(&c_st);
  }

  for (k = 1; k <= ny; k++) {
    if (k > y->size[0]) {
      emlrtDynamicBoundsCheckR2012b(k, 1, y->size[0], &x_emlrtBCI, &st);
    }

    yk = y->data[k - 1];
    if (muDoubleScalarIsNaN(yk)) {
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
          if ((nInflect < 1) || (nInflect > iInflect->size[0])) {
            emlrtDynamicBoundsCheckR2012b(nInflect, 1, iInflect->size[0],
              &bb_emlrtBCI, &st);
          }

          iInflect->data[nInflect - 1] = kfirst;
        }
      } else if (yk < ykfirst) {
        dir = 'd';
        if ('d' != previousdir) {
          nInflect++;
          if ((nInflect < 1) || (nInflect > iInflect->size[0])) {
            emlrtDynamicBoundsCheckR2012b(nInflect, 1, iInflect->size[0],
              &db_emlrtBCI, &st);
          }

          iInflect->data[nInflect - 1] = kfirst;
          if (previousdir == 'i') {
            nPk++;
            if ((nPk < 1) || (nPk > idx->size[0])) {
              emlrtDynamicBoundsCheckR2012b(nPk, 1, idx->size[0], &eb_emlrtBCI,
                &st);
            }

            idx->data[nPk - 1] = kfirst;
          }
        }
      } else {
        dir = 'i';
        if ('i' != previousdir) {
          nInflect++;
          if ((nInflect < 1) || (nInflect > iInflect->size[0])) {
            emlrtDynamicBoundsCheckR2012b(nInflect, 1, iInflect->size[0],
              &cb_emlrtBCI, &st);
          }

          iInflect->data[nInflect - 1] = kfirst;
        }
      }

      ykfirst = yk;
      kfirst = k;
      isinfykfirst = isinfyk;
    }
  }

  if ((y->size[0] > 0) && (!isinfykfirst)) {
    guard1 = false;
    if (nInflect == 0) {
      guard1 = true;
    } else {
      if (nInflect > iInflect->size[0]) {
        emlrtDynamicBoundsCheckR2012b(nInflect, 1, iInflect->size[0],
          &y_emlrtBCI, &st);
      }

      if (iInflect->data[nInflect - 1] < y->size[0]) {
        guard1 = true;
      }
    }

    if (guard1) {
      nInflect++;
      if ((nInflect < 1) || (nInflect > iInflect->size[0])) {
        emlrtDynamicBoundsCheckR2012b(nInflect, 1, iInflect->size[0],
          &ab_emlrtBCI, &st);
      }

      iInflect->data[nInflect - 1] = y->size[0];
    }
  }

  if (1 > nPk) {
    nPk = 0;
  } else {
    if (1 > idx->size[0]) {
      emlrtDynamicBoundsCheckR2012b(1, 1, idx->size[0], &t_emlrtBCI, &st);
    }

    if (nPk > idx->size[0]) {
      emlrtDynamicBoundsCheckR2012b(nPk, 1, idx->size[0], &u_emlrtBCI, &st);
    }
  }

  kfirst = idx->size[0];
  idx->size[0] = nPk;
  emxEnsureCapacity_int32_T(&st, idx, kfirst, &ab_emlrtRTEI);
  iInfinite->size[0] = 0;
  if (1 > nInflect) {
    nInflect = 0;
  } else {
    if (1 > iInflect->size[0]) {
      emlrtDynamicBoundsCheckR2012b(1, 1, iInflect->size[0], &v_emlrtBCI, &st);
    }

    if (nInflect > iInflect->size[0]) {
      emlrtDynamicBoundsCheckR2012b(nInflect, 1, iInflect->size[0], &w_emlrtBCI,
        &st);
    }
  }

  emxInit_int32_T(&st, &iPk, 1, &mb_emlrtRTEI, true);
  kfirst = iInflect->size[0];
  iInflect->size[0] = nInflect;
  emxEnsureCapacity_int32_T(&st, iInflect, kfirst, &bb_emlrtRTEI);
  st.site = &f_emlrtRSI;
  kfirst = iPk->size[0];
  iPk->size[0] = nPk;
  emxEnsureCapacity_int32_T(&st, iPk, kfirst, &cb_emlrtRTEI);
  ny = 0;
  b_st.site = &v_emlrtRSI;
  if ((1 <= nPk) && (nPk > 2147483646)) {
    c_st.site = &u_emlrtRSI;
    check_forloop_overflow_error(&c_st);
  }

  for (k = 0; k < nPk; k++) {
    kfirst = k + 1;
    if (kfirst > nPk) {
      emlrtDynamicBoundsCheckR2012b(kfirst, 1, nPk, &nb_emlrtBCI, &st);
    }

    if ((idx->data[k] < 1) || (idx->data[k] > y->size[0])) {
      emlrtDynamicBoundsCheckR2012b(idx->data[k], 1, y->size[0], &o_emlrtBCI,
        &st);
    }

    kfirst = idx->data[k] - 1;
    if ((kfirst < 1) || (kfirst > y->size[0])) {
      emlrtDynamicBoundsCheckR2012b(kfirst, 1, y->size[0], &p_emlrtBCI, &st);
    }

    kfirst = idx->data[k] + 1;
    if ((kfirst < 1) || (kfirst > y->size[0])) {
      emlrtDynamicBoundsCheckR2012b(kfirst, 1, y->size[0], &q_emlrtBCI, &st);
    }

    if (y->data[idx->data[k] - 1] - muDoubleScalarMax(y->data[idx->data[k] - 2],
         y->data[idx->data[k]]) >= 0.0) {
      ny++;
      if ((ny < 1) || (ny > iPk->size[0])) {
        emlrtDynamicBoundsCheckR2012b(ny, 1, iPk->size[0], &fb_emlrtBCI, &st);
      }

      iPk->data[ny - 1] = idx->data[k];
    }
  }

  if (1 > ny) {
    ny = 0;
  } else {
    if (1 > iPk->size[0]) {
      emlrtDynamicBoundsCheckR2012b(1, 1, iPk->size[0], &r_emlrtBCI, &st);
    }

    if (ny > iPk->size[0]) {
      emlrtDynamicBoundsCheckR2012b(ny, 1, iPk->size[0], &s_emlrtBCI, &st);
    }
  }

  emxInit_int32_T(&st, &b_iPk, 1, &qb_emlrtRTEI, true);
  kfirst = iPk->size[0];
  iPk->size[0] = ny;
  emxEnsureCapacity_int32_T(&st, iPk, kfirst, &db_emlrtRTEI);
  kfirst = b_iPk->size[0];
  b_iPk->size[0] = ny;
  emxEnsureCapacity_int32_T(sp, b_iPk, kfirst, &eb_emlrtRTEI);
  for (kfirst = 0; kfirst < ny; kfirst++) {
    b_iPk->data[kfirst] = iPk->data[kfirst];
  }

  emxInit_real_T(sp, &bPk, 1, &rb_emlrtRTEI, true);
  emxInit_real_T(sp, &bxPk, 2, &sb_emlrtRTEI, true);
  emxInit_real_T(sp, &byPk, 2, &tb_emlrtRTEI, true);
  emxInit_real_T(sp, &wxPk, 2, &ub_emlrtRTEI, true);
  st.site = &g_emlrtRSI;
  findExtents(&st, y, x, b_iPk, idx, iInfinite, iInflect, bPk, bxPk, byPk, wxPk);
  st.site = &h_emlrtRSI;
  c_findPeaksSeparatedByMoreThanM(&st, y, x, b_iPk, idx);
  st.site = &i_emlrtRSI;
  emxFree_int32_T(&iInflect);
  if (idx->size[0] > Np) {
    if (1 > idx->size[0]) {
      emlrtDynamicBoundsCheckR2012b(1, 1, idx->size[0], &m_emlrtBCI, &st);
    }

    kfirst = static_cast<int32_T>(Np);
    if ((kfirst < 1) || (kfirst > idx->size[0])) {
      emlrtDynamicBoundsCheckR2012b(kfirst, 1, idx->size[0], &n_emlrtBCI, &st);
    }

    iv[0] = 1;
    iv[1] = kfirst;
    b_st.site = &yd_emlrtRSI;
    indexShapeCheck(&b_st, idx->size[0], iv);
    ny = idx->size[0];
    idx->size[0] = kfirst;
    emxEnsureCapacity_int32_T(&st, idx, ny, &hb_emlrtRTEI);
  }

  kfirst = iPk->size[0];
  iPk->size[0] = idx->size[0];
  emxEnsureCapacity_int32_T(sp, iPk, kfirst, &fb_emlrtRTEI);
  ny = idx->size[0];
  for (kfirst = 0; kfirst < ny; kfirst++) {
    if ((idx->data[kfirst] < 1) || (idx->data[kfirst] > b_iPk->size[0])) {
      emlrtDynamicBoundsCheckR2012b(idx->data[kfirst], 1, b_iPk->size[0],
        &gb_emlrtBCI, sp);
    }

    iPk->data[kfirst] = b_iPk->data[idx->data[kfirst] - 1];
  }

  emxFree_int32_T(&b_iPk);
  st.site = &j_emlrtRSI;
  kfirst = iInfinite->size[0];
  iInfinite->size[0] = idx->size[0];
  emxEnsureCapacity_int32_T(&st, iInfinite, kfirst, &gb_emlrtRTEI);
  ny = idx->size[0];
  for (kfirst = 0; kfirst < ny; kfirst++) {
    if ((idx->data[kfirst] < 1) || (idx->data[kfirst] > bPk->size[0])) {
      emlrtDynamicBoundsCheckR2012b(idx->data[kfirst], 1, bPk->size[0],
        &hb_emlrtBCI, &st);
    }

    iInfinite->data[kfirst] = idx->data[kfirst];
  }

  kfirst = bPk->size[0];
  bPk->size[0] = iInfinite->size[0];
  emxEnsureCapacity_real_T(&st, bPk, kfirst, &ib_emlrtRTEI);
  ny = idx->size[0];
  for (kfirst = 0; kfirst < ny; kfirst++) {
    if ((idx->data[kfirst] < 1) || (idx->data[kfirst] > bxPk->size[0])) {
      emlrtDynamicBoundsCheckR2012b(idx->data[kfirst], 1, bxPk->size[0],
        &ib_emlrtBCI, &st);
    }
  }

  emxFree_real_T(&bxPk);
  ny = idx->size[0];
  for (kfirst = 0; kfirst < ny; kfirst++) {
    if ((idx->data[kfirst] < 1) || (idx->data[kfirst] > byPk->size[0])) {
      emlrtDynamicBoundsCheckR2012b(idx->data[kfirst], 1, byPk->size[0],
        &jb_emlrtBCI, &st);
    }
  }

  emxFree_real_T(&byPk);
  kfirst = iInfinite->size[0];
  iInfinite->size[0] = idx->size[0];
  emxEnsureCapacity_int32_T(&st, iInfinite, kfirst, &jb_emlrtRTEI);
  ny = idx->size[0];
  for (kfirst = 0; kfirst < ny; kfirst++) {
    if ((idx->data[kfirst] < 1) || (idx->data[kfirst] > wxPk->size[0])) {
      emlrtDynamicBoundsCheckR2012b(idx->data[kfirst], 1, wxPk->size[0],
        &kb_emlrtBCI, &st);
    }

    iInfinite->data[kfirst] = idx->data[kfirst];
  }

  emxFree_real_T(&wxPk);
  st.site = &k_emlrtRSI;
  ny = iPk->size[0];
  for (kfirst = 0; kfirst < ny; kfirst++) {
    if ((iPk->data[kfirst] < 1) || (iPk->data[kfirst] > y->size[0])) {
      emlrtDynamicBoundsCheckR2012b(iPk->data[kfirst], 1, y->size[0],
        &lb_emlrtBCI, &st);
    }
  }

  ny = iPk->size[0];
  for (kfirst = 0; kfirst < ny; kfirst++) {
    if ((iPk->data[kfirst] < 1) || (iPk->data[kfirst] > x->size[0])) {
      emlrtDynamicBoundsCheckR2012b(iPk->data[kfirst], 1, x->size[0],
        &mb_emlrtBCI, &st);
    }
  }

  b_st.site = &ae_emlrtRSI;
  if (iInfinite->size[0] != 0) {
    c_st.site = &be_emlrtRSI;
    if ((1 <= iInfinite->size[0]) && (iInfinite->size[0] > 2147483646)) {
      d_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&d_st);
    }
  }

  emxFree_int32_T(&iInfinite);
  if (idx->size[0] != bPk->size[0]) {
    emlrtSizeEqCheck1DR2012b(idx->size[0], bPk->size[0], &emlrtECI, &st);
  }

  emxFree_real_T(&bPk);
  emxFree_int32_T(&idx);
  kfirst = Ypk->size[0] * Ypk->size[1];
  Ypk->size[0] = 1;
  Ypk->size[1] = iPk->size[0];
  emxEnsureCapacity_real_T(&st, Ypk, kfirst, &kb_emlrtRTEI);
  ny = iPk->size[0];
  for (kfirst = 0; kfirst < ny; kfirst++) {
    Ypk->data[kfirst] = y->data[iPk->data[kfirst] - 1];
  }

  emxFree_real_T(&y);
  kfirst = Xpk->size[0] * Xpk->size[1];
  Xpk->size[0] = 1;
  Xpk->size[1] = iPk->size[0];
  emxEnsureCapacity_real_T(&st, Xpk, kfirst, &lb_emlrtRTEI);
  ny = iPk->size[0];
  for (kfirst = 0; kfirst < ny; kfirst++) {
    Xpk->data[kfirst] = x->data[iPk->data[kfirst] - 1];
  }

  emxFree_real_T(&x);
  emxFree_int32_T(&iPk);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (findpeaks.cpp) */
