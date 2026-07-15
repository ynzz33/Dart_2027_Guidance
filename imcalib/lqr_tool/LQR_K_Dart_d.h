/*
 * LQR weights used for this gain (discrete-time dlqr with Ts = 0.001 s):
 *   Q = diag([150.000, 1500.000, 2500.000, 1.000, 0.250, 0.950])
 *   R = diag([20.000, 20.000, 20.000, 20.000])
 * Fit range: 2.0 <= V <= 20.0 m/s, V_ref = 6.0 m/s
 */

/*
 * File: LQR_K_Dart_d.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2026-07-15 10:19:22
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
