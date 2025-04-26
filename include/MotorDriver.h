#pragma once
#include <Arduino.h>

class MotorDriver
{
public:
    /**
     * @param in1Pin  → L298 INx1
     * @param in2Pin  → L298 INx2
     */
    MotorDriver(uint8_t in1Pin, uint8_t in2Pin);

    /** Call in setup() to configure both LEDC channels */
    void begin(uint32_t freq = 20000, uint8_t resolution = 8);
    void brake();

    /**
     * @param speed –100…+100
     *   +: forward (drive on IN1),
     *   –: backward (drive on IN2),
     *    0: coast.
     */
    void setSpeed(int16_t speed);

private:
    uint8_t _in1Pin, _in2Pin;
    uint8_t _ch1, _ch2;
    uint8_t _resolution;
};
