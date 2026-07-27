/*
 * lqi_geometry_table.h — MATLAB 28-Jul-2026 06:10:24
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
    {-9.24563338e-02f, -9.11723847e-02f, -9.09247074e-02f, -9.22086565e-02f},
    {+7.03312431e-02f, -6.16887725e-02f, -6.20735914e-02f, +6.99464242e-02f},
    {+6.80575822e-02f, +6.76704406e-02f, -6.64550439e-02f, -6.60679022e-02f}
};

static const float lqi_N_ry[LQI_SERVO_COUNT][LQI_NRY_DIM] = {
    {-4.94035281e-01f, -5.01037871e-01f},
    {+5.03835536e-01f, +5.01058438e-01f},
    {+5.03131388e-01f, -4.98945242e-01f},
    {-4.98936569e-01f, +4.98954045e-01f}
};

#endif
