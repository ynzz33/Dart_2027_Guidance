/*
 * lqi_gain_table.h — MATLAB 25-Jul-2026 03:40:23
 * K_lqi 与速度无关(B_tau=I⁻¹不含气动)，存单矩阵。
 * Q=diag([20.00,0.050,110.0,1.0,1.3,1.0,0.00,0.0001,2.00])
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
    {+8.60893122e-01f, -2.60226690e-04f, -2.15603374e-03f, +1.93774034e-01f, -1.10269213e-03f, -2.32946495e-04f, +1.92404692e-03f, -9.63942688e-06f, -2.86514770e-04f},
    {-6.25878562e-03f, +7.61460622e-02f, +1.08814502e-03f, -1.46589524e-03f, +3.22083358e-01f, +1.46880007e-04f, -1.39877425e-05f, +2.82157943e-03f, +1.44077917e-04f},
    {-1.02245066e-03f, +1.89126860e-05f, +3.02796032e+00f, -2.41698439e-04f, +8.08415489e-05f, +3.01281584e-01f, -2.28506085e-06f, +6.99431091e-07f, +4.02848985e-01f}
};

#endif
