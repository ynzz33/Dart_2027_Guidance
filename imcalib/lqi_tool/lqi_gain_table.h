/*
 * lqi_gain_table.h — MATLAB 28-Jul-2026 06:10:24
 * K_lqi 与速度无关(B_tau=I⁻¹不含气动)，存单矩阵。
 * Q=diag([20.00,0.050,170.0,1.0,0.8,1.0,0.00,0.0001,1.00])
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
    {+9.00906923e-01f, -3.25214849e-04f, -2.35213386e-03f, +2.02865619e-01f, -1.12027034e-03f, -2.16089833e-04f, +2.01347505e-03f, -1.24750760e-05f, -1.79133736e-04f},
    {-3.25138026e-03f, +7.53401286e-02f, -2.89750686e-04f, -7.93354205e-04f, +2.59025130e-01f, +3.05942462e-05f, -7.26634138e-06f, +2.89085661e-03f, -2.24011308e-05f},
    {-9.72283289e-04f, +4.55831058e-05f, +3.73807019e+00f, -2.30495386e-04f, +1.57646325e-04f, +3.05320595e-01f, -2.17293927e-06f, +1.74748309e-06f, +2.84906518e-01f}
};

#endif
