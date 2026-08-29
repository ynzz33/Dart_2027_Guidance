/*
 * lqi_geometry_table.h — MATLAB 29-Aug-2026 17:40:13
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
    {-4.23261064e-02f, -4.40603576e-02f, -4.40564813e-02f, -4.23222301e-02f},
    {+3.42653964e-02f, -4.54135202e-02f, -4.54191425e-02f, +3.42597741e-02f},
    {+3.98501864e-02f, +3.98445301e-02f, -3.98251053e-02f, -3.98194491e-02f}
};

static const float lqi_N_ry[LQI_SERVO_COUNT][LQI_NRY_DIM] = {
    {+5.14934057e-01f, -4.94810726e-01f},
    {-4.94940027e-01f, +4.95013212e-01f},
    {-4.84735905e-01f, -5.04934727e-01f},
    {+5.04884529e-01f, +5.05138780e-01f}
};

#endif
