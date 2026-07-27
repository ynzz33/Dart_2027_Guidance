/*
 * lqi_gain_table.h — MATLAB 27-Jul-2026 22:58:22
 * K_lqi 与速度无关(B_tau=I⁻¹不含气动)，存单矩阵。
 * Q=diag([20.00,0.050,92.0,1.0,0.3,1.0,0.00,0.0001,1.00])
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
    {+8.60875019e-01f, -4.71639005e-04f, -1.96130099e-03f, +1.93769975e-01f, -1.05261055e-03f, -2.29783936e-04f, +1.92400646e-03f, -1.90919150e-05f, -2.01997272e-04f},
    {+6.29304113e-04f, +7.39218292e-02f, -2.24463695e-03f, +7.49666236e-05f, +1.64642151e-01f, -1.93527180e-04f, +1.40679189e-06f, +2.99295919e-03f, -2.31924362e-04f},
    {-1.01568740e-03f, +8.87810614e-05f, +2.76456282e+00f, -2.40177479e-04f, +1.98855341e-04f, +3.00008318e-01f, -2.26994551e-06f, +3.59256707e-06f, +2.84983085e-01f}
};

#endif
