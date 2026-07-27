/*
 * lqi_gain_table.h — MATLAB 28-Jul-2026 05:18:16
 * K_lqi 与速度无关(B_tau=I⁻¹不含气动)，存单矩阵。
 * Q=diag([30.00,0.050,90.0,1.0,0.6,1.0,0.00,0.0001,1.00])
 * R=diag([10.0,10.0,10.0]) V_ref=6.0 Ts=0.001s
 */

#ifndef LQI_GAIN_TABLE_H
#define LQI_GAIN_TABLE_H
#include <stdint.h>
#include "lqi_torque.h"  /* LQI_STATE_DIM, LQI_TORQUE_DIM */

#define LQI_V_REF 6.0000f
#define LQI_V_MIN 1.0000f
#define LQI_V_MAX 20.0000f

static const float lqi_K[LQI_TORQUE_DIM][LQI_STATE_DIM] = {
    {+1.05366254e+00f, -3.89187834e-04f, -1.96957823e-03f, +1.93982402e-01f, -1.18245315e-03f, -2.32604555e-04f, +1.92306930e-03f, -1.52000324e-05f, -2.05043266e-04f},
    {-2.61251819e-03f, +7.49033979e-02f, -7.21074033e-04f, -5.57603143e-04f, +2.27176489e-01f, -3.59193671e-05f, -4.76792539e-06f, +2.92610182e-03f, -7.56080080e-05f},
    {-1.22712879e-03f, +5.64688418e-05f, +2.73514790e+00f, -2.40048865e-04f, +1.72265635e-04f, +2.99865929e-01f, -2.23962039e-06f, +2.20422436e-06f, +2.84997311e-01f}
};

#endif
