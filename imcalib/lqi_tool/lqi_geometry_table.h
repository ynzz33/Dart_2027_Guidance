/*
 * lqi_geometry_table.h — MATLAB 29-Jul-2026 08:57:27
 * H_tau(V) = (V/V_ref)^2 * lqi_H_tau_Vref
 * N_ry: H_ry*N_ry=0 (4x2) ⚠ 占位符
 */

#ifndef LQI_GEOMETRY_TABLE_H
#define LQI_GEOMETRY_TABLE_H
#include <stdint.h>
#include "lqi_gain_table.h"  /* LQI_V_REF */

#define LQI_NRY_DIM 2
#define LQI_PITCH_ROW 1
#define LQI_DELTA_MAX_DEG 60.0000f
#define LQI_DELTA_MAX_RAD 1.04719755f
#define LQI_LAMBDA_PITCH 1.0000f
#define LQI_LAMBDA_SERVO 1.0000f
#define LQI_GAIN_SCALAR 1.0f

static const float lqi_H_tau_Vref[3][4] = {
    {-8.34533132e-02f, -8.21693641e-02f, -8.19216868e-02f, -8.32056359e-02f},
    {+7.04712353e-02f, -6.18287647e-02f, -6.22135836e-02f, +7.00864164e-02f},
    {+6.71448400e-02f, +6.67576983e-02f, -6.55423017e-02f, -6.51551600e-02f}
};

static const float lqi_N_ry[LQI_SERVO_COUNT][LQI_NRY_DIM] = {
    {-4.93453387e-01f, -5.01217685e-01f},
    {+5.04044866e-01f, +5.01243789e-01f},
    {+5.03677317e-01f, -4.98760154e-01f},
    {-4.98750126e-01f, +4.98772298e-01f}
};

#endif
