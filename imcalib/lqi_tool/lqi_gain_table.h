/*
 * lqi_gain_table.h — MATLAB 28-Jul-2026 06:04:53
 * K_lqi 与速度无关(B_tau=I⁻¹不含气动)，存单矩阵。
 * Q=diag([20.00,0.050,140.0,1.0,0.6,1.0,0.00,0.0001,1.00])
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
    {+8.60886055e-01f, -3.86887955e-04f, -2.37660279e-03f, +1.93772449e-01f, -1.17549187e-03f, -2.33024354e-04f, +1.92403112e-03f, -1.51101632e-05f, -1.99201973e-04f},
    {-2.19552105e-03f, +7.49033914e-02f, -1.00997259e-03f, -5.57141153e-04f, +2.27176470e-01f, -3.71273527e-05f, -4.90654969e-06f, +2.92610157e-03f, -8.50924404e-05f},
    {-1.03957467e-03f, +5.88615646e-05f, +3.39392826e+00f, -2.45531091e-04f, +1.79507891e-04f, +3.03043014e-01f, -2.32333218e-06f, +2.29772160e-06f, +2.84682865e-01f}
};

#endif
