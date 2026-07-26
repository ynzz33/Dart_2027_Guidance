/*
 * lqi_gain_table.h — MATLAB 26-Jul-2026 20:40:12
 * K_lqi 与速度无关(B_tau=I⁻¹不含气动)，存单矩阵。
 * Q=diag([20.00,0.050,100.0,1.0,0.1,0.8,0.00,0.0001,200.00])
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
    {+8.60856823e-01f, -5.62486734e-04f, -2.56331416e-03f, +1.93765892e-01f, -7.60298224e-04f, -2.36755571e-04f, +1.92396579e-03f, -2.36804893e-05f, -3.18218469e-03f},
    {+3.77506247e-03f, +7.27327309e-02f, -4.82415693e-03f, +7.79737772e-04f, +9.80261342e-02f, -3.47107440e-04f, +8.43739799e-06f, +3.06255737e-03f, -6.15983806e-03f},
    {-8.24945457e-04f, +1.27312628e-04f, +3.24346596e+00f, -1.97564499e-04f, +1.72901979e-04f, +2.76535477e-01f, -1.84364664e-06f, +5.35827915e-06f, +4.06710550e+00f}
};

#endif
