#include "MotorDriver.h"

MotorDriver::MotorDriver(uint8_t in1Pin, uint8_t in2Pin)
    : _in1Pin(in1Pin), _in2Pin(in2Pin), _resolution(8)
{
    static uint8_t nextChannel = 0;
    _ch1 = nextChannel++;
    _ch2 = nextChannel++;
}

void MotorDriver::begin(uint32_t freq, uint8_t resolution)
{
    _resolution = resolution;
    // Channel 1 → IN1 pin
    ledcSetup(_ch1, freq, resolution);
    ledcAttachPin(_in1Pin, _ch1);
    // Channel 2 → IN2 pin
    ledcSetup(_ch2, freq, resolution);
    ledcAttachPin(_in2Pin, _ch2);
}

void MotorDriver::setSpeed(int16_t speed)
{
    // map 0–100 → 0–(2^resolution–1)
    uint32_t maxDuty = (1u << _resolution) - 1;
    uint32_t duty = min(abs(speed), 100) * maxDuty / 100;

    if (speed > 0)
    {
        // FORWARD: PWM on IN1, IN2=0 (coast/brake off)
        ledcWrite(_ch1, duty);
        ledcWrite(_ch2, 0);
    }
    else if (speed < 0)
    {
        // BACKWARD: PWM on IN2, IN1=0
        ledcWrite(_ch1, 0);
        ledcWrite(_ch2, duty);
    }
    else
    {
        // SPEED == 0 → coast (both channels 0)
        ledcWrite(_ch1, 0);
        ledcWrite(_ch2, 0);
    }
}

void MotorDriver::brake()
{
    uint32_t maxDuty = (1u << _resolution) - 1;
    // both high = short‐brake
    ledcWrite(_ch1, maxDuty);
    ledcWrite(_ch2, maxDuty);
}