/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: matlab_inv.h
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 24-Sep-2022 19:18:17
 */

#ifndef MATLAB_INV_H
#define MATLAB_INV_H

/* Include Files */
#include "ikid_motion_control/rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void matlab_inv(const double a[36], double result[36]);
extern void matlab_inv_8(const double a[64], double b[64]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for matlab_inv.h
 *
 * [EOF]
 */
