/*
 * lqi_gain_table.h — MATLAB 30-Aug-2026 16:51:51
 * K_lqi 与速度无关(B_tau=I⁻¹不含气动)，存单矩阵。
 * Q=diag([70.00,0.010,500.0,1.0,0.5,1.0,0.00,0.0001,1.00])
 * R=diag([5.0,5.0,5.0]) V_ref=6.0 Ts=0.001s
 */

#ifndef LQI_GAIN_TABLE_H
#define LQI_GAIN_TABLE_H
#include <stdint.h>
#include "lqi_torque.h"  /* LQI_STATE_DIM, LQI_TORQUE_DIM */

#define LQI_V_REF 6.0000f
#define LQI_V_MIN 1.0000f
#define LQI_V_MAX 20.0000f

static const float lqi_K[LQI_TORQUE_DIM][LQI_STATE_DIM] = {
    {+1.19624684e+00f, +1.01074343e-03f, -2.78947509e-04f, +1.44294352e-01f, +4.60300902e-03f, -1.34567374e-05f, +1.42958233e-03f, +6.50320174e-05f, -1.24480273e-05f},
    {+2.78593798e-02f, +5.88891617e-02f, -1.14493959e-05f, +3.49784330e-03f, +2.68146457e-01f, +8.27964529e-07f, +3.32933311e-05f, +3.78920664e-03f, -5.13680567e-07f},
    {-1.09058309e-04f, +1.71221216e-06f, +7.86374390e+00f, -1.34195991e-05f, +7.80260916e-06f, +3.71767957e-01f, -1.30330187e-07f, +1.10135557e-07f, +3.50934569e-01f}
};

#endif
