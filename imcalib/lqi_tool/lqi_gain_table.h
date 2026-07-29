/*
 * lqi_gain_table.h — MATLAB 29-Jul-2026 08:57:27
 * K_lqi 与速度无关(B_tau=I⁻¹不含气动)，存单矩阵。
 * Q=diag([40.00,0.050,390.0,1.0,0.4,1.0,0.00,0.0001,1.00])
 * R=diag([10.0,10.0,8.0]) V_ref=6.0 Ts=0.001s
 */

#ifndef LQI_GAIN_TABLE_H
#define LQI_GAIN_TABLE_H
#include <stdint.h>
#include "lqi_torque.h"  /* LQI_STATE_DIM, LQI_TORQUE_DIM */

#define LQI_V_REF 6.0000f
#define LQI_V_MIN 1.0000f
#define LQI_V_MAX 20.0000f

static const float lqi_K[LQI_TORQUE_DIM][LQI_STATE_DIM] = {
    {+1.21604194e+00f, -4.43639089e-04f, -4.82330380e-03f, +1.94157781e-01f, -1.12644297e-03f, -2.98319257e-04f, +1.92224580e-03f, -1.77147556e-05f, -2.43473743e-04f},
    {-4.80499191e-04f, +7.43088411e-02f, -3.01979278e-03f, -1.67788312e-04f, +1.88325812e-01f, -8.16134144e-05f, -7.59315873e-07f, +2.96781757e-03f, -1.52703789e-04f},
    {-1.49477713e-03f, +8.12217641e-05f, +6.20707664e+00f, -2.54901248e-04f, +2.06876167e-04f, +3.43471102e-01f, -2.36281303e-06f, +3.24208304e-06f, +3.13428082e-01f}
};

#endif
