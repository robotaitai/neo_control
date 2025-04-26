// src/fl_ramp_dynamic_brake.cpp
#include <Arduino.h>
#include "Config.h"
#include "MotorDriver.h"
#include "Encoder.h"
#include "Logger.h"

// Front-Left only
MotorDriver flMotor(FL_PWM_PIN, FL_DIR_PIN);
Encoder flEnc(FL_ENC_A_PIN, FL_ENC_B_PIN);

void dynamicBrake(int16_t brakeSpeed = 100, uint16_t durationMs = 200)
{
  Logger::info("[FL] Dynamic brake: reverse %d%% for %dms",
               brakeSpeed, durationMs);
  flMotor.setSpeed(-brakeSpeed);
  delay(durationMs);
  flMotor.setSpeed(0);
}

void setup()
{
  Logger::begin(DBG_BAUD);
  flMotor.begin();
  flEnc.begin();

  Logger::info("=== FL Ramp + Dynamic Brake Test Start ===");

  const int step = 20;     // % per increment
  const int holdMs = 2000; // hold time
  const int logMs = 500;   // log every 500 ms

  // Sweep from -100 → +100
  for (int sp = -100; sp <= 100; sp += step)
  {
    Logger::info("[FL] setSpeed = %4d%%", sp);
    flMotor.setSpeed(sp);
    long lastPos = flEnc.read();
    uint32_t t0 = millis();

    while (millis() - t0 < holdMs)
    {
      delay(logMs);
      long now = flEnc.read();
      long d = now - lastPos;
      lastPos = now;
      float vel = (float)d * 1000.0f / logMs;
      Logger::info("[FL] t=%4lums pos=%6ld vel=%6.1f c/s",
                   millis() - t0, now, vel);
    }
  }

  // Coast briefly
  Logger::info("=== FL Coast ===");
  flMotor.setSpeed(0);
  delay(200);

  // Dynamic brake hack
  dynamicBrake(/*brakeSpeed*/ 100, /*durationMs*/ 300);

  Logger::info("=== FL Ramp + Dynamic Brake Test Complete ===");
}

void loop()
{
  // done
  while (true)
    delay(1000);
}
