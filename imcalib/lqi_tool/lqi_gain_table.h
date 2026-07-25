/*
 * lqi_gain_table.h — MATLAB 26-Jul-2026 07:39:34
 * K_lqi 与速度无关(B_tau=I⁻¹不含气动)，存单矩阵。
 * Q=diag([20.00,0.050,100.0,1.0,0.2,1.0,0.00,0.0001,30.00])
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
    {+8.60868090e-01f, -5.10599851e-04f, -2.12018695e-03f, +1.93768421e-01f, -9.48212115e-04f, -2.29924415e-04f, +1.92399097e-03f, -2.10169069e-05f, -1.09433080e-03f},
    {+1.96211525e-03f, +7.34296977e-02f, -3.35693778e-03f, +3.73401021e-04f, +1.36049600e-01f, -2.80631872e-04f, +4.38555645e-06f, +3.02303039e-03f, -1.75624073e-03f},
    {-1.02472578e-03f, +1.06923918e-04f, +3.00911133e+00f, -2.42207262e-04f, +1.99272241e-04f, +3.01187664e-01f, -2.29014566e-06f, +4.39981683e-06f, +1.56027678e+00f}
};

#endif
