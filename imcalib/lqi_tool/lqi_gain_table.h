/*
 * lqi_gain_table.h — MATLAB 01-Aug-2026 01:59:08
 * K_lqi 与速度无关(B_tau=I⁻¹不含气动)，存单矩阵。
 * Q=diag([30.00,0.050,700.0,1.0,0.3,1.0,0.00,0.0001,0.00])
 * R=diag([10.0,10.0,6.0]) V_ref=6.0 Ts=0.001s
 */

#ifndef LQI_GAIN_TABLE_H
#define LQI_GAIN_TABLE_H
#include <stdint.h>
#include "lqi_torque.h"  /* LQI_STATE_DIM, LQI_TORQUE_DIM */

#define LQI_V_REF 6.0000f
#define LQI_V_MIN 1.0000f
#define LQI_V_MAX 20.0000f

static const float lqi_K[LQI_TORQUE_DIM][LQI_STATE_DIM] = {
    {+1.05364920e+00f, -4.73909924e-04f, -8.15823017e-03f, +1.93979957e-01f, -1.05764867e-03f, -3.80483449e-04f, +1.92304496e-03f, -1.91838960e-05f, -3.08346640e-06f},
    {+8.47465288e-04f, +7.39218461e-02f, -4.36041910e-03f, +7.50640269e-05f, +1.64642188e-01f, -7.80266453e-05f, +1.54700261e-06f, +2.99295987e-03f, -1.64807264e-06f},
    {-1.26543018e-03f, +9.51672985e-05f, +9.38413420e+00f, -2.47138515e-04f, +2.13023196e-04f, +3.94507460e-01f, -2.30952522e-06f, +3.85123479e-06f, +3.54681299e-03f}
};

#endif
