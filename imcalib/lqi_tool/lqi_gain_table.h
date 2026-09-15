/*
 * lqi_gain_table.h — MATLAB 03-Sep-2026 21:57:38
 * K_lqi 与速度无关(B_tau=I⁻¹不含气动)，存单矩阵。
 * Q=diag([50.00,0.010,200.0,1.0,0.5,1.0,0.00,0.0001,1.00])
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
    {+1.01172557e+00f, +1.00951919e-03f, -1.78781112e-04f, +1.44180882e-01f, +4.59743764e-03f, -1.32240022e-05f, +1.43050769e-03f, +6.49532263e-05f, -1.25757866e-05f},
    {+2.37087943e-02f, +5.88891333e-02f, -5.56877926e-07f, +3.49528776e-03f, +2.68146328e-01f, +8.45765929e-07f, +3.35223085e-05f, +3.78920481e-03f, -4.35848659e-08f},
    {-9.07172580e-05f, +1.64338158e-06f, +5.00835111e+00f, -1.31535941e-05f, +7.48936698e-06f, +3.65675669e-01f, -1.28267340e-07f, +1.05705604e-07f, +3.52320245e-01f}
};

#endif
