/*
 * lqi_gain_table.h — MATLAB 27-Jul-2026 06:47:56
 * K_lqi 与速度无关(B_tau=I⁻¹不含气动)，存单矩阵。
 * Q=diag([20.00,0.050,90.0,1.0,0.1,0.3,0.00,0.0001,5.00])
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
    {+8.60846930e-01f, -5.99666304e-04f, -3.07432224e-03f, +1.93763666e-01f, -5.89721650e-04f, -2.28240607e-04f, +1.92394368e-03f, -2.56643687e-05f, -7.12054899e-04f},
    {+5.10324565e-03f, +7.22024032e-02f, -4.62543166e-03f, +1.07798496e-03f, +7.07397925e-02f, -2.35168029e-04f, +1.14058155e-05f, +3.09060806e-03f, -1.07720777e-03f},
    {-1.44171743e-04f, +1.28130534e-04f, +2.85694209e+00f, -4.53772427e-05f, +1.27352584e-04f, +1.95096069e-01f, -3.22150988e-07f, +5.48112013e-06f, +6.62636260e-01f}
};

#endif
