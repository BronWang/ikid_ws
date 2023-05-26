/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: matlab_inv.c
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 24-Sep-2022 19:18:17
 */

/* Include Files */
#include "ros_socket/matlab_inv.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : const double a[36]
 *                double result[36]
 * Return Type  : void
 */
void matlab_inv(const double a[36], double result[36])
{
  double x[36];
  double s;
  double smax;
  int b_i;
  int b_tmp;
  int i;
  int j;
  int jA;
  int jp1j;
  int k;
  int kAcol;
  int mmj_tmp;
  int x_tmp;
  signed char ipiv[6];
  signed char p[6];
  signed char i1;
  for (i = 0; i < 36; i++) {
    result[i] = 0.0;
    x[i] = a[i];
  }
  for (i = 0; i < 6; i++) {
    ipiv[i] = (signed char)(i + 1);
  }
  for (j = 0; j < 5; j++) {
    mmj_tmp = 4 - j;
    b_tmp = j * 7;
    jp1j = b_tmp + 2;
    jA = 6 - j;
    kAcol = 0;
    smax = fabs(x[b_tmp]);
    for (k = 2; k <= jA; k++) {
      s = fabs(x[(b_tmp + k) - 1]);
      if (s > smax) {
        kAcol = k - 1;
        smax = s;
      }
    }
    if (x[b_tmp + kAcol] != 0.0) {
      if (kAcol != 0) {
        jA = j + kAcol;
        ipiv[j] = (signed char)(jA + 1);
        for (k = 0; k < 6; k++) {
          kAcol = j + k * 6;
          smax = x[kAcol];
          x_tmp = jA + k * 6;
          x[kAcol] = x[x_tmp];
          x[x_tmp] = smax;
        }
      }
      i = (b_tmp - j) + 6;
      for (b_i = jp1j; b_i <= i; b_i++) {
        x[b_i - 1] /= x[b_tmp];
      }
    }
    jA = b_tmp;
    for (kAcol = 0; kAcol <= mmj_tmp; kAcol++) {
      smax = x[(b_tmp + kAcol * 6) + 6];
      if (smax != 0.0) {
        i = jA + 8;
        jp1j = (jA - j) + 12;
        for (x_tmp = i; x_tmp <= jp1j; x_tmp++) {
          x[x_tmp - 1] += x[((b_tmp + x_tmp) - jA) - 7] * -smax;
        }
      }
      jA += 6;
    }
  }
  for (i = 0; i < 6; i++) {
    p[i] = (signed char)(i + 1);
  }
  for (k = 0; k < 5; k++) {
    i1 = ipiv[k];
    if (i1 > k + 1) {
      jA = p[i1 - 1];
      p[i1 - 1] = p[k];
      p[k] = (signed char)jA;
    }
  }
  for (k = 0; k < 6; k++) {
    x_tmp = 6 * (p[k] - 1);
    result[k + x_tmp] = 1.0;
    for (j = k + 1; j < 7; j++) {
      i = (j + x_tmp) - 1;
      if (result[i] != 0.0) {
        jp1j = j + 1;
        for (b_i = jp1j; b_i < 7; b_i++) {
          jA = (b_i + x_tmp) - 1;
          result[jA] -= result[i] * x[(b_i + 6 * (j - 1)) - 1];
        }
      }
    }
  }
  for (j = 0; j < 6; j++) {
    jA = 6 * j;
    for (k = 5; k >= 0; k--) {
      kAcol = 6 * k;
      i = k + jA;
      smax = result[i];
      if (smax != 0.0) {
        result[i] = smax / x[k + kAcol];
        for (b_i = 0; b_i < k; b_i++) {
          x_tmp = b_i + jA;
          result[x_tmp] -= result[i] * x[b_i + kAcol];
        }
      }
    }
  }
}

void matlab_inv_8(const double a[64], double b[64])
{
  double x[64];
  double s;
  double smax;
  int b_tmp;
  int i;
  int j;
  int jA;
  int jp1j;
  int k;
  int kAcol;
  int mmj_tmp;
  int temp_tmp;
  signed char ipiv[8];
  signed char p[8];
  signed char i1;
  for (i = 0; i < 64; i++) {
    b[i] = 0.0;
    x[i] = a[i];
  }
  for (i = 0; i < 8; i++) {
    ipiv[i] = (signed char)(i + 1);
  }
  for (j = 0; j < 7; j++) {
    mmj_tmp = 6 - j;
    b_tmp = j * 9;
    jp1j = b_tmp + 2;
    jA = 8 - j;
    kAcol = 0;
    smax = fabs(x[b_tmp]);
    for (k = 2; k <= jA; k++) {
      s = fabs(x[(b_tmp + k) - 1]);
      if (s > smax) {
        kAcol = k - 1;
        smax = s;
      }
    }
    if (x[b_tmp + kAcol] != 0.0) {
      if (kAcol != 0) {
        kAcol += j;
        ipiv[j] = (signed char)(kAcol + 1);
        for (k = 0; k < 8; k++) {
          jA = k << 3;
          temp_tmp = j + jA;
          smax = x[temp_tmp];
          jA += kAcol;
          x[temp_tmp] = x[jA];
          x[jA] = smax;
        }
      }
      i = (b_tmp - j) + 8;
      for (temp_tmp = jp1j; temp_tmp <= i; temp_tmp++) {
        x[temp_tmp - 1] /= x[b_tmp];
      }
    }
    jA = b_tmp;
    for (kAcol = 0; kAcol <= mmj_tmp; kAcol++) {
      smax = x[(b_tmp + (kAcol << 3)) + 8];
      if (smax != 0.0) {
        i = jA + 10;
        jp1j = (jA - j) + 16;
        for (temp_tmp = i; temp_tmp <= jp1j; temp_tmp++) {
          x[temp_tmp - 1] += x[((b_tmp + temp_tmp) - jA) - 9] * -smax;
        }
      }
      jA += 8;
    }
  }
  for (i = 0; i < 8; i++) {
    p[i] = (signed char)(i + 1);
  }
  for (k = 0; k < 7; k++) {
    i1 = ipiv[k];
    if (i1 > k + 1) {
      jA = p[i1 - 1];
      p[i1 - 1] = p[k];
      p[k] = (signed char)jA;
    }
  }
  for (k = 0; k < 8; k++) {
    b_tmp = (p[k] - 1) << 3;
    b[k + b_tmp] = 1.0;
    for (j = k + 1; j < 9; j++) {
      i = (j + b_tmp) - 1;
      if (b[i] != 0.0) {
        jp1j = j + 1;
        for (temp_tmp = jp1j; temp_tmp < 9; temp_tmp++) {
          jA = (temp_tmp + b_tmp) - 1;
          b[jA] -= b[i] * x[(temp_tmp + ((j - 1) << 3)) - 1];
        }
      }
    }
  }
  for (j = 0; j < 8; j++) {
    jA = j << 3;
    for (k = 7; k >= 0; k--) {
      kAcol = k << 3;
      i = k + jA;
      smax = b[i];
      if (smax != 0.0) {
        b[i] = smax / x[k + kAcol];
        for (temp_tmp = 0; temp_tmp < k; temp_tmp++) {
          b_tmp = temp_tmp + jA;
          b[b_tmp] -= b[i] * x[temp_tmp + kAcol];
        }
      }
    }
  }
}


/*
 * File trailer for matlab_inv.c
 *
 * [EOF]
 */
