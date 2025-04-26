#pragma once
#include "Encoder.h"
#include <stdint.h>
#include <math.h>

/**
 * Differential‐drive odometry using four encoders
 * (front‐left, rear‐left → left; front‐right, rear‐right → right).
 */
class Odometry
{
public:
    Odometry(Encoder &fl, Encoder &rl,
             Encoder &fr, Encoder &rr,
             float wheelRadius,
             float trackWidth,
             uint32_t ticksPerRev);

    /** Call once in setup(), passing the initial millis(). */
    void begin(uint32_t t0_ms);

    /**
     * Call at fixed rate (e.g. ODOM_HZ) with current millis().
     * Integrates encoder ticks into x,y,θ and v,ω.
     */
    void update(uint32_t now_ms);

    // getters
    float x() const { return _x; }
    float y() const { return _y; }
    float theta() const { return _theta; }
    float linVel() const { return _v; }
    float angVel() const { return _omega; }

private:
    Encoder &_fl;
    Encoder &_rl;
    Encoder &_fr;
    Encoder &_rr;

    float _wheelRadius;
    float _trackWidth;
    float _metersPerTick;

    int32_t _prevLeftCount;
    int32_t _prevRightCount;
    uint32_t _prevTime;

    float _x;
    float _y;
    float _theta;

    float _v;
    float _omega;
};
