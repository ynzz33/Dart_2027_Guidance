/*
 * File: LQR_K_Dart_d_initialize.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2026-07-08 14:56:40
 */

/* Include Files */
#include "LQR_K_Dart_d_initialize.h"
#include "LQR_K_Dart_d_data.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : void
 */
void LQR_K_Dart_d_initialize(void)
{
  rt_InitInfAndNaN();
  isInitialized_LQR_K_Dart_d = true;
}

/*
 * File trailer for LQR_K_Dart_d_initialize.c
 *
 * [EOF]
 */
