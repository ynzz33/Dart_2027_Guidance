/*
 * lqi_gain_table.h — MATLAB 08-Aug-2026 08:41:33
 * K_lqi 与速度无关(B_tau=I⁻¹不含气动)，存单矩阵。
 * Q=diag([10.00,0.010,500.0,1.0,0.5,1.0,0.00,0.0001,1.00])
 * R=diag([5.0,5.0,1.0]) V_ref=6.0 Ts=0.001s
 */

#ifndef LQI_GAIN_TABLE_H
#define LQI_GAIN_TABLE_H
#include <stdint.h>
#include "lqi_torque.h"  /* LQI_STATE_DIM, LQI_TORQUE_DIM */

#define LQI_V_REF 6.0000f
#define LQI_V_MIN 1.0000f
#define LQI_V_MAX 20.0000f

static const float lqi_K[LQI_TORQUE_DIM][LQI_STATE_DIM] = {
    {+7.19086097e-01f, -3.90624914e-04f, -2.12168570e-02f, +2.28137098e-01f, -1.78007684e-03f, -1.01121402e-03f, +2.27166967e-03f, -2.51264415e-05f, -9.46826420e-04f},
    {-2.75025175e-03f, +6.28944218e-02f, +7.07692026e-03f, -9.16495491e-04f, +2.86471618e-01f, +4.06850217e-04f, -8.68789958e-06f, +4.04641120e-03f, +3.15676832e-04f},
    {-7.60182997e-04f, +1.27539903e-05f, +1.62622134e+01f, -2.49538516e-04f, +5.87605646e-05f, +7.60537201e-01f, -2.40141513e-06f, +8.16631237e-07f, +7.25748810e-01f}
};

#endif
