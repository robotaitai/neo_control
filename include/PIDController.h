#pragma once
#include <Arduino.h> // for millis()
#include <stdint.h>

class PIDController
{
public:
    PIDController(float kp = 0, float ki = 0, float kd = 0);

    /**
     * Call once per loop.
     * @param setpoint Desired value.
     * @param measurement Current value.
     * @return control output (–100…+100).
     */
    float update(float setpoint, float measurement);

    void reset();
    void setTunings(float kp, float ki, float kd);

private:
    float _kp, _ki, _kd;
    float _integral;
    float _lastError;
    uint32_t _lastTime; // <-- now known
};
