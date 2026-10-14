/*
 * lqi_gain_table.h — MATLAB 13-Oct-2026 22:48:16
 * K_lqi 与速度无关(B_tau=I⁻¹不含气动)，存单矩阵。
 * Q=diag([50.00,0.010,1000.0,1.0,0.5,1.0,0.00,0.0001,1.00])
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
    {+1.01172557e+00f, +1.00951919e-03f, -3.88491836e-04f, +1.44180882e-01f, +4.59743762e-03f, -1.36743892e-05f, +1.43050769e-03f, +6.49532259e-05f, -1.22715236e-05f},
    {+2.37087943e-02f, +5.88891333e-02f, -3.19586509e-05f, +3.49528776e-03f, +2.68146328e-01f, +7.94103170e-07f, +3.35223086e-05f, +3.78920481e-03f, -1.01141189e-06f},
    {-9.45121616e-05f, +1.78581782e-06f, +1.10610778e+01f, -1.36908282e-05f, +8.13755665e-06f, +3.78498614e-01f, -1.33633230e-07f, +1.14872694e-07f, +3.49403697e-01f}
};

#endif
