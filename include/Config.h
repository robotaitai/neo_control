#pragma once

// ── Right Side (FL & FR) ─────────────────────
// ── Front wheels ─────────────────────────────
// IN1/IN2 on the board drive Front-Right
constexpr uint8_t FR_PWM_PIN = 18; // IN1 → PWM for Front-Right
constexpr uint8_t FR_DIR_PIN = 19; // IN2 → DIR for Front-Right
constexpr bool FR_INVERT = true;

// IN3/IN4 on the board drive Front-Left
constexpr uint8_t FL_PWM_PIN = 16; // IN3 → PWM for Front-Left
constexpr uint8_t FL_DIR_PIN = 17; // IN4 → DIR for Front-Left

// ── Left Side (RL & RR) ──────────────────────
constexpr uint8_t RL_PWM_PIN = 25; // IN1→PWM   Rear-Left
constexpr uint8_t RL_DIR_PIN = 26; // IN2→DIR

constexpr uint8_t RR_PWM_PIN = 14; // IN3→PWM   Rear-Right
constexpr uint8_t RR_DIR_PIN = 27; // IN4→DIR

// ── Encoders (yellow = A, white = B) ────────
constexpr uint8_t FL_ENC_A_PIN = 23; // FL A (yellow)
constexpr uint8_t FL_ENC_B_PIN = 22; // FL B (white)

constexpr uint8_t FR_ENC_A_PIN = 21; // FR A (yellow)
constexpr uint8_t FR_ENC_B_PIN = 2;  // FR B (white)

constexpr uint8_t RL_ENC_A_PIN = 34; // RL A (yellow)
constexpr uint8_t RL_ENC_B_PIN = 35; // RL B (white)

constexpr uint8_t RR_ENC_A_PIN = 32; // RR A (yellow)
constexpr uint8_t RR_ENC_B_PIN = 33; // RR B (white)

// ── Control & comm ──────────────────────────
constexpr uint32_t CONTROL_HZ = 100;
constexpr uint32_t CONTROL_PERIOD = 1000 / CONTROL_HZ;
constexpr float PID_KP = 1.0f;
constexpr float PID_KI = 0.1f;
constexpr float PID_KD = 0.01f;

constexpr uint32_t DBG_BAUD = 115200;
constexpr uint32_t UP_BAUD = 1500000;

constexpr uint8_t UP_RX_PIN = 5;
constexpr uint8_t UP_TX_PIN = 4;
