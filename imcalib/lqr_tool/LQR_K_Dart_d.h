/*
 * LQR weights used for this gain (discrete-time dlqr with Ts = 0.001 s):
 *   Q = diag([20.000, 10.000, 300.000, 1.000, 1.000, 0.700])
 *   R = diag([20.000, 20.000, 20.000, 20.000])
 * Fit range: 1.0 <= V <= 20.0 m/s, V_ref = 6.0 m/s
 */

/*
 * File: LQR_K_Dart_d.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2026-07-20 18:22:46
 */

#ifndef LQR_K_DART_D_H
#define LQR_K_DART_D_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void LQR_K_Dart_d(double V, double K_d_sym[24]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for LQR_K_Dart_d.h
 *
 * [EOF]
 */
