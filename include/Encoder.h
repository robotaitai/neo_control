// include/Encoder.h
#pragma once
#include <Arduino.h>
#include <stdint.h>

class Encoder
{
public:
    Encoder(uint8_t pinA, uint8_t pinB);
    void begin();
    long read();

private:
    static void IRAM_ATTR isrA(void *arg);
    static void IRAM_ATTR isrB(void *arg);
    void handleA();
    void handleB();

    volatile long _count;
    uint8_t _pinA, _pinB;
};
