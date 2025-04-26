#pragma once
#include "MechConfig.h" // for TRACK_WIDTH_M, MAX_LINEAR_SPEED_MPS

// ── Odometry ──────────────────────────────────
constexpr uint32_t ODOM_HZ = 50;                    // integration rate (Hz)
constexpr uint32_t ODOM_PERIOD_MS = 1000 / ODOM_HZ; // ms per update

// ── Velocity control loop ─────────────────────
constexpr uint32_t VEL_CTRL_HZ = 100; // PID update rate (Hz)
constexpr uint32_t VEL_CTRL_PERIOD_MS = 1000 / VEL_CTRL_HZ;

// PID gains for wheel‐velocity (tune as needed)
constexpr float VEL_PID_KP = 1.0f;
constexpr float VEL_PID_KI = 0.1f;
constexpr float VEL_PID_KD = 0.01f;

// ── Body‐frame limits ─────────────────────────
// max linear and angular speeds for feed-forward / safety
constexpr float MAX_LIN_VEL_MPS = MAX_LINEAR_SPEED_MPS;
// for differential‐style odom: ω_max ≈ 2·v_max / track_width
constexpr float MAX_ANG_VEL_RADPS = (2.0f * MAX_LINEAR_SPEED_MPS) / TRACK_WIDTH_M;
