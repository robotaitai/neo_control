// src/Encoder.cpp
#include "Encoder.h"

Encoder::Encoder(uint8_t pinA, uint8_t pinB)
    : _pinA(pinA), _pinB(pinB), _count(0) {}

void Encoder::begin()
{
    pinMode(_pinA, INPUT_PULLUP);
    pinMode(_pinB, INPUT_PULLUP);

    // use attachInterruptArg (ESP32 core) to pass 'this'
    attachInterruptArg(_pinA, isrA, this, CHANGE);
    attachInterruptArg(_pinB, isrB, this, CHANGE);
}

long Encoder::read()
{
    noInterrupts();
    long v = _count;
    interrupts();
    return v;
}

// static ISR trampolines:
void IRAM_ATTR Encoder::isrA(void *arg)
{
    static_cast<Encoder *>(arg)->handleA();
}
void IRAM_ATTR Encoder::isrB(void *arg)
{
    static_cast<Encoder *>(arg)->handleB();
}

// quadrature decode:
void Encoder::handleA()
{
    bool A = digitalRead(_pinA);
    bool B = digitalRead(_pinB);
    _count += (A == B) ? +1 : -1;
}
void Encoder::handleB()
{
    bool A = digitalRead(_pinA);
    bool B = digitalRead(_pinB);
    _count += (A != B) ? +1 : -1;
}
