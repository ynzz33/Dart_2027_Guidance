/*
 * lqi_gain_table.h — MATLAB 31-Aug-2026 22:29:13
 * K_lqi 与速度无关(B_tau=I⁻¹不含气动)，存单矩阵。
 * Q=diag([80.00,0.010,600.0,1.0,0.5,1.0,0.00,0.0001,1.00])
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
    {+1.27845001e+00f, +1.01128620e-03f, -3.04740063e-04f, +1.44344896e-01f, +4.60547910e-03f, -1.35194967e-05f, +1.42917014e-03f, +6.50669496e-05f, -1.24184484e-05f},
    {+2.96915955e-02f, +5.88891743e-02f, -1.54351567e-05f, +3.49897145e-03f, +2.68146514e-01f, +8.21205155e-07f, +3.31918397e-05f, +3.78920745e-03f, -6.31495719e-07f},
    {-1.16976177e-04f, +1.72948387e-06f, +8.60262143e+00f, -1.34896857e-05f, +7.88120434e-06f, +3.73331780e-01f, -1.30765816e-07f, +1.11247141e-07f, +3.50578881e-01f}
};

#endif
