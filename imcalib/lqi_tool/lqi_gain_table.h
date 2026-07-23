/*
 * lqi_gain_table.h — MATLAB 24-Jul-2026 00:09:10
 * K_lqi 与速度无关(B_tau=I⁻¹不含气动)，存单矩阵。
 * Q=diag([50.00,0.050,600.0,0.1,0.0,0.1,5.00,5.0000,0.50])
 * R=diag([10.0,10.0,100.0]) V_ref=6.0 Ts=0.001s
 */

#ifndef LQI_GAIN_TABLE_H
#define LQI_GAIN_TABLE_H
#include <stdint.h>
#include "lqi_torque.h"  /* LQI_STATE_DIM, LQI_TORQUE_DIM */

#define LQI_V_REF 6.0000f
#define LQI_V_MIN 1.0000f
#define LQI_V_MAX 20.0000f

static const float lqi_K[LQI_TORQUE_DIM][LQI_STATE_DIM] = {
    {+1.93682373e+00f, -1.97337499e-03f, +1.96879033e-02f, +8.38409024e-02f, -3.86933725e-04f, +2.54106929e-04f, +6.04150736e-01f, -4.71823694e-03f, +5.68129017e-04f},
    {+1.10691284e-02f, +2.80415683e-01f, -5.01970842e-02f, +2.22997619e-04f, +5.30844912e-02f, -1.28432238e-03f, +3.47785755e-03f, +6.95127398e-01f, -1.44799470e-03f},
    {-2.22237468e-03f, +6.22544230e-04f, +2.38165990e+00f, -1.48328883e-04f, +1.21396247e-04f, +9.12521193e-02f, -6.88091674e-04f, +1.49768885e-03f, +6.86765978e-02f}
};

#endif
