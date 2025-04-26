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
    
    // incoming twist command: <V,lin,ang>\n
    static bool receiveTwist(float &lin, float &ang);

    // outgoing odometry: <O,x,y,th,v,om>\n
    static void sendOdometry(float x, float y, float th, float v, float om);

private:
    static HardwareSerial *_up;
};
