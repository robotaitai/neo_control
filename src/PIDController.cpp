#include "PIDController.h"
#include <Arduino.h>

PIDController::PIDController(float kp, float ki, float kd)
    : _kp(kp), _ki(ki), _kd(kd),
      _integral(0), _lastError(0),
      _lastTime(millis()) {}

void PIDController::setTunings(float kp, float ki, float kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void PIDController::reset()
{
    _integral = 0;
    _lastError = 0;
    _lastTime = millis();
}

float PIDController::update(float setpoint, float measurement)
{
    uint32_t now = millis();
    float dt = (now - _lastTime) / 1000.0f;
    if (dt <= 0)
        return 0;

    float error = setpoint - measurement;
    _integral += error * dt;
    float derivative = (error - _lastError) / dt;

    float output = _kp * error + _ki * _integral + _kd * derivative;

    _lastError = error;
    _lastTime = now;

    // Saturate to –100…+100
    if (output > 100)
        output = 100;
    if (output < -100)
        output = -100;
    return output;
}
