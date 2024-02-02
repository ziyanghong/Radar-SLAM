/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * eml_setop.cpp
 *
 * Code generation for function 'eml_setop'
 *
 */

/* Include files */
#include "eml_setop.h"
#include "extractPointFromPolar.h"
#include "extractPointFromPolar_emxutil.h"
#include "issorted.h"
#include "mwmathutil.h"
#include "rt_nonfinite.h"

/* Variable Definitions */
static emlrtRSInfo fc_emlrtRSI = { 224,/* lineNo */
  "do_vectors",                        /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/ops/private/eml_setop.m"/* pathName */
};

static emlrtRSInfo gc_emlrtRSI = { 227,/* lineNo */
  "do_vectors",                        /* fcnName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/ops/private/eml_setop.m"/* pathName */
};

static emlrtRTEInfo j_emlrtRTEI = { 225,/* lineNo */
  13,                                  /* colNo */
  "do_vectors",                        /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/ops/private/eml_setop.m"/* pName */
};

static emlrtRTEInfo k_emlrtRTEI = { 228,/* lineNo */
  13,                                  /* colNo */
  "do_vectors",                        /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/ops/private/eml_setop.m"/* pName */
};

static emlrtRTEInfo l_emlrtRTEI = { 392,/* lineNo */
  5,                                   /* colNo */
  "do_vectors",                        /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/ops/private/eml_setop.m"/* pName */
};

static emlrtRTEInfo m_emlrtRTEI = { 403,/* lineNo */
  9,                                   /* colNo */
  "do_vectors",                        /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/ops/private/eml_setop.m"/* pName */
};

static emlrtRTEInfo n_emlrtRTEI = { 430,/* lineNo */
  5,                                   /* colNo */
  "do_vectors",                        /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/ops/private/eml_setop.m"/* pName */
};

static emlrtRTEInfo gd_emlrtRTEI = { 134,/* lineNo */
  22,                                  /* colNo */
  "eml_setop",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/ops/private/eml_setop.m"/* pName */
};

static emlrtRTEInfo hd_emlrtRTEI = { 398,/* lineNo */
  9,                                   /* colNo */
  "eml_setop",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/ops/private/eml_setop.m"/* pName */
};

static emlrtRTEInfo id_emlrtRTEI = { 409,/* lineNo */
  13,                                  /* colNo */
  "eml_setop",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/ops/private/eml_setop.m"/* pName */
};

static emlrtRTEInfo jd_emlrtRTEI = { 432,/* lineNo */
  9,                                   /* colNo */
  "eml_setop",                         /* fName */
  "/usr/local/MATLAB/R2019b/toolbox/eml/lib/matlab/ops/private/eml_setop.m"/* pName */
};

/* Function Definitions */
void b_do_vectors(const emlrtStack *sp, const emxArray_int32_T *a, const
                  emxArray_int32_T *b, emxArray_int32_T *c, emxArray_int32_T *ia,
                  emxArray_int32_T *ib)
{
  int32_T na;
  int32_T nb;
  int32_T ncmax;
  int32_T iafirst;
  int32_T nc;
  int32_T ialast;
  int32_T ibfirst;
  int32_T iblast;
  int32_T b_ialast;
  int32_T ak;
  int32_T b_iblast;
  int32_T bk;
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  na = a->size[0];
  nb = b->size[0];
  ncmax = muIntScalarMin_sint32(na, nb);
  iafirst = c->size[0];
  c->size[0] = ncmax;
  emxEnsureCapacity_int32_T(sp, c, iafirst, &gd_emlrtRTEI);
  iafirst = ia->size[0];
  ia->size[0] = ncmax;
  emxEnsureCapacity_int32_T(sp, ia, iafirst, &gd_emlrtRTEI);
  iafirst = ib->size[0];
  ib->size[0] = ncmax;
  emxEnsureCapacity_int32_T(sp, ib, iafirst, &gd_emlrtRTEI);
  st.site = &fc_emlrtRSI;
  if (!issorted(&st, a)) {
    emlrtErrorWithMessageIdR2018a(sp, &j_emlrtRTEI,
      "Coder:toolbox:eml_setop_unsortedA", "Coder:toolbox:eml_setop_unsortedA",
      0);
  }

  st.site = &gc_emlrtRSI;
  if (!issorted(&st, b)) {
    emlrtErrorWithMessageIdR2018a(sp, &k_emlrtRTEI,
      "Coder:toolbox:eml_setop_unsortedB", "Coder:toolbox:eml_setop_unsortedB",
      0);
  }

  nc = 0;
  iafirst = 0;
  ialast = 1;
  ibfirst = 0;
  iblast = 1;
  while ((ialast <= na) && (iblast <= nb)) {
    b_ialast = ialast;
    ak = a->data[ialast - 1];
    while ((b_ialast < a->size[0]) && (a->data[b_ialast] == ak)) {
      b_ialast++;
    }

    ialast = b_ialast;
    b_iblast = iblast;
    bk = b->data[iblast - 1];
    while ((b_iblast < b->size[0]) && (b->data[b_iblast] == bk)) {
      b_iblast++;
    }

    iblast = b_iblast;
    if (ak == bk) {
      nc++;
      c->data[nc - 1] = ak;
      ia->data[nc - 1] = iafirst + 1;
      ib->data[nc - 1] = ibfirst + 1;
      ialast = b_ialast + 1;
      iafirst = b_ialast;
      iblast = b_iblast + 1;
      ibfirst = b_iblast;
    } else if (ak < bk) {
      ialast = b_ialast + 1;
      iafirst = b_ialast;
    } else {
      iblast = b_iblast + 1;
      ibfirst = b_iblast;
    }
  }

  if (ncmax > 0) {
    if (nc > ncmax) {
      emlrtErrorWithMessageIdR2018a(sp, &l_emlrtRTEI,
        "Coder:builtins:AssertionFailed", "Coder:builtins:AssertionFailed", 0);
    }

    iafirst = ia->size[0];
    if (1 > nc) {
      ia->size[0] = 0;
    } else {
      ia->size[0] = nc;
    }

    emxEnsureCapacity_int32_T(sp, ia, iafirst, &hd_emlrtRTEI);
    if (nc > ncmax) {
      emlrtErrorWithMessageIdR2018a(sp, &m_emlrtRTEI,
        "Coder:builtins:AssertionFailed", "Coder:builtins:AssertionFailed", 0);
    }

    iafirst = ib->size[0];
    if (1 > nc) {
      ib->size[0] = 0;
    } else {
      ib->size[0] = nc;
    }

    emxEnsureCapacity_int32_T(sp, ib, iafirst, &id_emlrtRTEI);
    if (nc > ncmax) {
      emlrtErrorWithMessageIdR2018a(sp, &n_emlrtRTEI,
        "Coder:builtins:AssertionFailed", "Coder:builtins:AssertionFailed", 0);
    }

    iafirst = c->size[0];
    if (1 > nc) {
      c->size[0] = 0;
    } else {
      c->size[0] = nc;
    }

    emxEnsureCapacity_int32_T(sp, c, iafirst, &jd_emlrtRTEI);
  }
}

void do_vectors(const emlrtStack *sp, const emxArray_int32_T *a, const
                emxArray_int32_T *b, emxArray_int32_T *c, emxArray_int32_T *ia,
                emxArray_int32_T *ib)
{
  int32_T na;
  int32_T nb;
  int32_T ncmax;
  int32_T iafirst;
  int32_T nc;
  int32_T nia;
  int32_T nib;
  int32_T ialast;
  int32_T ibfirst;
  int32_T iblast;
  int32_T b_ialast;
  int32_T ak;
  int32_T b_iblast;
  int32_T bk;
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  na = a->size[0];
  nb = b->size[0];
  ncmax = a->size[0] + b->size[0];
  iafirst = c->size[0];
  c->size[0] = ncmax;
  emxEnsureCapacity_int32_T(sp, c, iafirst, &gd_emlrtRTEI);
  iafirst = ia->size[0];
  ia->size[0] = a->size[0];
  emxEnsureCapacity_int32_T(sp, ia, iafirst, &gd_emlrtRTEI);
  iafirst = ib->size[0];
  ib->size[0] = b->size[0];
  emxEnsureCapacity_int32_T(sp, ib, iafirst, &gd_emlrtRTEI);
  st.site = &fc_emlrtRSI;
  if (!issorted(&st, a)) {
    emlrtErrorWithMessageIdR2018a(sp, &j_emlrtRTEI,
      "Coder:toolbox:eml_setop_unsortedA", "Coder:toolbox:eml_setop_unsortedA",
      0);
  }

  st.site = &gc_emlrtRSI;
  if (!issorted(&st, b)) {
    emlrtErrorWithMessageIdR2018a(sp, &k_emlrtRTEI,
      "Coder:toolbox:eml_setop_unsortedB", "Coder:toolbox:eml_setop_unsortedB",
      0);
  }

  nc = -1;
  nia = 0;
  nib = 0;
  iafirst = 1;
  ialast = 1;
  ibfirst = 0;
  iblast = 1;
  while ((ialast <= na) && (iblast <= nb)) {
    b_ialast = ialast;
    ak = a->data[ialast - 1];
    while ((b_ialast < a->size[0]) && (a->data[b_ialast] == ak)) {
      b_ialast++;
    }

    ialast = b_ialast;
    b_iblast = iblast;
    bk = b->data[iblast - 1];
    while ((b_iblast < b->size[0]) && (b->data[b_iblast] == bk)) {
      b_iblast++;
    }

    iblast = b_iblast;
    if (ak == bk) {
      nc++;
      c->data[nc] = ak;
      nia++;
      ia->data[nia - 1] = iafirst;
      ialast = b_ialast + 1;
      iafirst = b_ialast + 1;
      iblast = b_iblast + 1;
      ibfirst = b_iblast;
    } else if (ak < bk) {
      nc++;
      nia++;
      c->data[nc] = ak;
      ia->data[nia - 1] = iafirst;
      ialast = b_ialast + 1;
      iafirst = b_ialast + 1;
    } else {
      nc++;
      nib++;
      c->data[nc] = bk;
      ib->data[nib - 1] = ibfirst + 1;
      iblast = b_iblast + 1;
      ibfirst = b_iblast;
    }
  }

  while (ialast <= na) {
    b_ialast = ialast;
    while ((b_ialast < a->size[0]) && (a->data[b_ialast] == a->data[ialast - 1]))
    {
      b_ialast++;
    }

    nc++;
    nia++;
    c->data[nc] = a->data[ialast - 1];
    ia->data[nia - 1] = ialast;
    ialast = b_ialast + 1;
  }

  while (iblast <= nb) {
    b_iblast = iblast;
    while ((b_iblast < b->size[0]) && (b->data[b_iblast] == b->data[iblast - 1]))
    {
      b_iblast++;
    }

    nc++;
    nib++;
    c->data[nc] = b->data[iblast - 1];
    ib->data[nib - 1] = iblast;
    iblast = b_iblast + 1;
  }

  if (a->size[0] > 0) {
    if (nia > a->size[0]) {
      emlrtErrorWithMessageIdR2018a(sp, &l_emlrtRTEI,
        "Coder:builtins:AssertionFailed", "Coder:builtins:AssertionFailed", 0);
    }

    iafirst = ia->size[0];
    if (1 > nia) {
      ia->size[0] = 0;
    } else {
      ia->size[0] = nia;
    }

    emxEnsureCapacity_int32_T(sp, ia, iafirst, &hd_emlrtRTEI);
  }

  if (b->size[0] > 0) {
    if (nib > b->size[0]) {
      emlrtErrorWithMessageIdR2018a(sp, &m_emlrtRTEI,
        "Coder:builtins:AssertionFailed", "Coder:builtins:AssertionFailed", 0);
    }

    iafirst = ib->size[0];
    if (1 > nib) {
      ib->size[0] = 0;
    } else {
      ib->size[0] = nib;
    }

    emxEnsureCapacity_int32_T(sp, ib, iafirst, &id_emlrtRTEI);
  }

  if (ncmax > 0) {
    if (nc + 1 > ncmax) {
      emlrtErrorWithMessageIdR2018a(sp, &n_emlrtRTEI,
        "Coder:builtins:AssertionFailed", "Coder:builtins:AssertionFailed", 0);
    }

    iafirst = c->size[0];
    if (1 > nc + 1) {
      c->size[0] = 0;
    } else {
      c->size[0] = nc + 1;
    }

    emxEnsureCapacity_int32_T(sp, c, iafirst, &jd_emlrtRTEI);
  }
}

/* End of code generation (eml_setop.cpp) */
