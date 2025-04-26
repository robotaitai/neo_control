// src/rl_ramp_dynamic_brake.cpp
#include <Arduino.h>
#include "Config.h"
#include "MotorDriver.h"
#include "Encoder.h"
#include "Logger.h"

// Rear-Left only
MotorDriver rlMotor(RL_PWM_PIN, RL_DIR_PIN);
Encoder rlEnc(RL_ENC_A_PIN, RL_ENC_B_PIN);

// reverse briefly to kill inertia
void dynamicBrake(int16_t brakeSp = 100, uint16_t dur = 200)
{
  Logger::info("[RL] Dynamic brake: reverse %d%% for %dms", brakeSp, dur);
  rlMotor.setSpeed(-brakeSp);
  delay(dur);
  rlMotor.setSpeed(0);
}

void setup()
{
  Logger::begin(DBG_BAUD);
  rlMotor.begin();
  rlEnc.begin();

  Logger::info("=== RL Ramp + Dynamic Brake Test Start ===");

  const int step = 20;     // % per increment
  const int holdMs = 2000; // ms to hold each speed
  const int logMs = 500;   // ms between logs

  // sweep -100 → +100
  for (int sp = -100; sp <= 100; sp += step)
  {
    Logger::info("[RL] setSpeed = %4d%%", sp);
    rlMotor.setSpeed(sp);
    long lastPos = rlEnc.read();
    uint32_t t0 = millis();

    while (millis() - t0 < holdMs)
    {
      delay(logMs);
      long now = rlEnc.read();
      long d = now - lastPos;
      lastPos = now;
      float vel = (float)d * 1000.0f / logMs;
      Logger::info(
          "[RL] t=%4lums pos=%6ld vel=%6.1f c/s",
          millis() - t0, now, vel);
    }
  }

  // coast briefly
  Logger::info("=== RL Coast ===");
  rlMotor.setSpeed(0);
  delay(200);

  // dynamic brake
  dynamicBrake(/*sp*/ 100, /*ms*/ 300);

  Logger::info("=== RL Ramp + Dynamic Brake Test Complete ===");
}

void loop()
{
  while (true)
    delay(1000);
}
