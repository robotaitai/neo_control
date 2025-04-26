#pragma once
#include <Arduino.h>
#include <stdint.h>

struct State
{
    uint32_t timestamp;
    int32_t fl_pos;
    int32_t rl_pos;
    float fl_vel;
    float rl_vel;
};

struct Command
{
    float fl_set; // –100…+100
    float rl_set;
};

class CommProtocol
{
public:
    /** UART to Jetson */
    static void begin(HardwareSerial &port, uint32_t baud);

    /** send current state */
    static void sendState(const State &s);

    /** try to read a command; returns true if a full packet arrived */
    static bool receiveCmd(Command &cmd);

private:
    static HardwareSerial *_up;
};
