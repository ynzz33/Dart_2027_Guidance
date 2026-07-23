/*
 * File: _coder_LQR_K_Dart_d_api.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2026-07-23 18:08:11
 */

#ifndef _CODER_LQR_K_DART_D_API_H
#define _CODER_LQR_K_DART_D_API_H

/* Include Files */
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"
#include <string.h>

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void LQR_K_Dart_d(real_T V, real_T K_d_sym[24]);

void LQR_K_Dart_d_api(const mxArray *prhs, const mxArray **plhs);

void LQR_K_Dart_d_atexit(void);

void LQR_K_Dart_d_initialize(void);

void LQR_K_Dart_d_terminate(void);

void LQR_K_Dart_d_xil_shutdown(void);

void LQR_K_Dart_d_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_LQR_K_Dart_d_api.h
 *
 * [EOF]
 */
