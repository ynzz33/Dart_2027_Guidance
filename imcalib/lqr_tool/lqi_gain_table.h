/* Pure LQR torque gain: 6 states -> 3 torques, no integral.
 * State: [e_roll,e_pitch,e_yaw,p,q,r], measured - target.
 * Units: angle rad, rate rad/s, torque N*m.
 * Torque rows: [roll,pitch,yaw]; tau = -K_tau*x.
 * All three angle errors participate; no pitch-angle zeroing.
 * Existing inertia values/signs retained; physical axis mapping needs bench confirmation.
 * Ideal torque model only: no servo polarity or effectiveness in K.
 * Code gain table: existing 3x9 MCU LQI must be adapted before use.
 * MCU must supply e_pitch = measured pitch - target pitch, in rad.
 * Q states [e_roll,e_pitch,e_yaw,p,q,r]: 50 0.01 1000 1 0.5 1
 * R torque: 5 5 5
 */

#ifndef DART_LQR_TORQUE_GAIN_TABLE_H
#define DART_LQR_TORQUE_GAIN_TABLE_H

#define DART_LQR_STATE_DIM 6
#define DART_LQR_TORQUE_DIM 3
#define DART_LQR_DESIGN_TS_S 1.000000000e-03f

static const float dart_lqr_K_tau[DART_LQR_TORQUE_DIM][DART_LQR_STATE_DIM] = {
    {+1.011522293e+00f, +6.496055867e-04f, -3.880797885e-04f, +1.441807300e-01f, +4.596333951e-03f, -1.367360892e-05f},
    {+2.369490266e-02f, +3.789353371e-02f, -3.207312693e-05f, +3.493960947e-03f, +2.680822909e-01f, +7.887140896e-07f},
    {-9.448632773e-05f, +1.149024001e-06f, +1.104928970e+01f, -1.368984431e-05f, +8.134379641e-06f, +3.784742951e-01f}
};

#endif /* DART_LQR_TORQUE_GAIN_TABLE_H */
