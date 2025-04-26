#pragma once

// ── Wheel geometry ────────────────────────────
// Mecanum wheel diameter is 97 mm
constexpr float WHEEL_DIAMETER_M = 0.097f;
constexpr float WHEEL_RADIUS_M = WHEEL_DIAMETER_M / 2.0f;
constexpr float WHEEL_CIRCUMFERENCE_M = WHEEL_DIAMETER_M * 3.141592653589793f;

// ── Encoder & gearbox ─────────────────────────
// 12 CPR on motor side, 34:1 gearbox
constexpr uint16_t ENCODER_CPR_MOTOR = 12;                                // pulses per motor rev
constexpr uint8_t GEARBOX_RATIO = 34;                                     // motor→wheel
constexpr uint32_t ENCODER_CPR_WHEEL = ENCODER_CPR_MOTOR * GEARBOX_RATIO; // pulses per wheel rev
constexpr uint32_t ENCODER_TICKS_PER_REV = ENCODER_CPR_WHEEL * 4;         // 4× quadrature decoding

// ── Chassis dimensions ────────────────────────
// track width = distance between left/right wheel centers
// wheelbase   = distance between front/rear wheel centers
constexpr float TRACK_WIDTH_M = 0.20f;
constexpr float WHEEL_BASE_M = 0.20f;

// ── Speed limits ──────────────────────────────
// nominal no-load motor speed is 294 RPM (chassis spec)
constexpr float MAX_WHEEL_RPM = 294.0f;
// convert to linear (m/s): rev/s = RPM/60
constexpr float MAX_LINEAR_SPEED_MPS = (MAX_WHEEL_RPM / 60.0f) * WHEEL_CIRCUMFERENCE_M;
