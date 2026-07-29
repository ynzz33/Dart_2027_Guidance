/*
 * lqi_gain_table.h — MATLAB 30-Jul-2026 00:33:50
 * K_lqi 与速度无关(B_tau=I⁻¹不含气动)，存单矩阵。
 * Q=diag([20.00,0.050,650.0,1.0,0.6,1.0,0.00,0.0001,1.00])
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
    {+8.60886217e-01f, -3.86890072e-04f, -5.93545961e-03f, +1.93772485e-01f, -1.17549828e-03f, -3.01256064e-04f, +1.92403148e-03f, -1.51102459e-05f, -2.32344863e-04f},
    {-2.19576566e-03f, +7.49033954e-02f, -2.01616682e-03f, -5.57195993e-04f, +2.27176482e-01f, +1.12155184e-05f, -4.90709618e-06f, +2.92610173e-03f, -7.90976381e-05f},
    {-1.13845690e-03f, +6.53612243e-05f, +7.98378426e+00f, -2.67672744e-04f, +1.99180973e-04f, +3.50869864e-01f, -2.54432899e-06f, +2.55169999e-06f, +3.12610448e-01f}
};

#endif
