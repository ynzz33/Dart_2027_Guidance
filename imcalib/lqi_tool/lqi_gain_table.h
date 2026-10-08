/*
 * lqi_gain_table.h — MATLAB 08-Oct-2026 21:13:44
 * K_lqi 与速度无关(B_tau=I⁻¹不含气动)，存单矩阵。
 * Q=diag([50.00,0.010,950.0,1.0,0.5,1.0,0.00,0.0001,1.00])
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
    {+1.01172557e+00f, +1.00951919e-03f, -3.79097123e-04f, +1.44180882e-01f, +4.59743762e-03f, -1.36543723e-05f, +1.43050769e-03f, +6.49532259e-05f, -1.22851705e-05f},
    {+2.37087943e-02f, +5.88891333e-02f, -2.98594347e-05f, +3.49528776e-03f, +2.68146328e-01f, +7.97636208e-07f, +3.35223084e-05f, +3.78920481e-03f, -9.69605358e-07f},
    {-9.43422848e-05f, +1.77963370e-06f, +1.07856385e+01f, -1.36667915e-05f, +8.10941549e-06f, +3.77922488e-01f, -1.33392644e-07f, +1.14474699e-07f, +3.49534735e-01f}
};

#endif
