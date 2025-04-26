// Logger.h
#pragma once
#include <Arduino.h>
class Logger
{
public:
    static void begin(uint32_t baud = 115200) { Serial.begin(baud); }
    template <typename... Args>
    static void info(const char *fmt, Args... args)
    {
        Serial.printf("[INFO] ");
        Serial.printf(fmt, args...);
        Serial.println();
    }
};
