/*
 * lqi_gain_table.h — MATLAB 27-Jul-2026 01:48:14
 * K_lqi 与速度无关(B_tau=I⁻¹不含气动)，存单矩阵。
 * Q=diag([20.00,0.050,200.0,1.0,0.4,0.8,0.00,0.0001,200.00])
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
    {+8.60879841e-01f, -4.39444324e-04f, -3.34373710e-03f, +1.93771056e-01f, -1.11584350e-03f, -2.43270901e-04f, +1.92401723e-03f, -1.75471651e-05f, -3.10921023e-03f},
    {-4.55230846e-04f, +7.43088377e-02f, -2.39741210e-03f, -1.67768667e-04f, +1.88325803e-01f, -8.43195272e-05f, -1.01708901e-06f, +2.96781743e-03f, -2.31298030e-03f},
    {-8.72628149e-04f, +7.24457137e-05f, +4.33216309e+00f, -2.08240040e-04f, +1.84700876e-04f, +2.82263643e-01f, -1.95021480e-06f, +2.89145876e-06f, +4.05908520e+00f}
};

#endif
