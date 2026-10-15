/* Pure LQR torque gain: 6 states -> 3 torques, no integral.
 * State: [e_roll,e_pitch,e_yaw,p,q,r], measured - target.
 * Units: angle rad, rate rad/s, torque N*m.
 * Torque rows: [roll,pitch,yaw]; tau = -K_tau*x.
 * Pitch angle column is zero; pitch rate damping is retained.
 * Existing inertia values/signs retained; physical axis mapping needs bench confirmation.
 * Ideal torque model only: no servo polarity or effectiveness in K.
 * Handoff attachment: existing 3x9 MCU LQI must be adapted before use.
 * Q active states [e_roll,e_yaw,p,q,r]: 50 1000 1 0.5 1
 * R torque: 5 5 5
 */

#ifndef DART_LQR_TORQUE_GAIN_TABLE_H
#define DART_LQR_TORQUE_GAIN_TABLE_H

#define DART_LQR_STATE_DIM 6
#define DART_LQR_TORQUE_DIM 3
#define DART_LQR_DESIGN_TS_S 1.000000000e-03f

static const float dart_lqr_K_tau[DART_LQR_TORQUE_DIM][DART_LQR_STATE_DIM] = {
    {+1.011521935e+00f, +0.000000000e+00f, -3.880843869e-04f, +1.441807002e-01f, +4.594352096e-03f, -1.367377808e-05f},
    {+2.367841825e-02f, +0.000000000e+00f, -3.234137694e-05f, +3.491570475e-03f, +2.679666877e-01f, +7.788546554e-07f},
    {-9.448682977e-05f, +0.000000000e+00f, +1.104928970e+01f, -1.368991707e-05f, +8.130873539e-06f, +3.784742951e-01f}
};

#endif /* DART_LQR_TORQUE_GAIN_TABLE_H */
