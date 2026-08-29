/*
 * lqi_gain_table.h — MATLAB 29-Aug-2026 17:40:13
 * K_lqi 与速度无关(B_tau=I⁻¹不含气动)，存单矩阵。
 * Q=diag([100.00,0.010,500.0,1.0,0.5,1.0,0.00,0.0001,1.00])
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
    {+1.42856195e+00f, +1.01227321e-03f, -2.79424046e-04f, +1.44437187e-01f, +4.60997090e-03f, -1.34786460e-05f, +1.42841752e-03f, +6.51304732e-05f, -1.24692953e-05f},
    {+3.30108210e-02f, +5.88891970e-02f, -1.14602647e-05f, +3.50101528e-03f, +2.68146618e-01f, +8.27463702e-07f, +3.30073202e-05f, +3.78920891e-03f, -5.14183148e-07f},
    {-1.29727193e-04f, +1.71207765e-06f, +7.86374390e+00f, -1.34323198e-05f, +7.80197509e-06f, +3.71767957e-01f, -1.29713473e-07f, +1.10126658e-07f, +3.50934569e-01f}
};

#endif
