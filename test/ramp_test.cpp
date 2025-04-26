// src/ramp_test.cpp
#include <Arduino.h>
#include "Config.h"
#include "MotorDriver.h"
#include "Encoder.h"
#include "Logger.h"

// Only FL motor + encoder for this test:
MotorDriver flMotor(FL_PWM_PIN, FL_DIR_PIN);
Encoder flEnc(FL_ENC_A_PIN, FL_ENC_B_PIN);

void setup()
{
    Logger::begin(DBG_BAUD);
    flMotor.begin();
    flEnc.begin();

    Logger::info("=== FL Ramp Test Start ===");
}

void loop()
{
    const int step = 20;     // change per step
    const int holdMs = 2000; // hold each speed for 2 s
    const int logInterval = 500;

    // Ramp from 0 to +100
    for (int sp = 0; sp <= 100; sp += step)
    {
        Logger::info("Setting speed = %d", sp);
        flMotor.setSpeed(sp);
        long prevCount = flEnc.read();
        uint32_t t0 = millis();
        while (millis() - t0 < holdMs)
        {
            delay(logInterval);
            long now = millis();
            long cnt = flEnc.read();
            long dCount = cnt - prevCount;
            prevCount = cnt;
            float vel = (float)dCount * 1000.0f / logInterval;
            Logger::info("  t=%lums, dCount=%ld → vel=%.1f counts/s",
                         now - t0, dCount, vel);
        }
    }

    // Ramp from +100 down to –100
    for (int sp = 100; sp >= -100; sp -= step)
    {
        Logger::info("Setting speed = %d", sp);
        flMotor.setSpeed(sp);
        long prevCount = flEnc.read();
        uint32_t t0 = millis();
        while (millis() - t0 < holdMs)
        {
            delay(logInterval);
            long now = millis();
            long cnt = flEnc.read();
            long dCount = cnt - prevCount;
            prevCount = cnt;
            float vel = (float)dCount * 1000.0f / logInterval;
            Logger::info("  t=%lums, dCount=%ld → vel=%.1f counts/s",
                         now - t0, dCount, vel);
        }
    }

    // Back to zero
    flMotor.setSpeed(0);
    Logger::info("=== FL Ramp Test Complete ===");

    while (true)
    {
        delay(1000);
    } // done
}
