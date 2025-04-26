#include "Odometry.h"
#include <math.h>

Odometry::Odometry(Encoder &fl, Encoder &rl,
                   Encoder &fr, Encoder &rr,
                   float wheelRadius, float trackWidth,
                   uint32_t ticksPerRev)
    : _fl(fl), _rl(rl), _fr(fr), _rr(rr),
      _wheelRadius(wheelRadius),
      _trackWidth(trackWidth),
      _metersPerTick((2.0f * wheelRadius) * (float)M_PI / ticksPerRev),
      _prevLeftCount(0), _prevRightCount(0),
      _prevTime(0),
      _x(0), _y(0), _theta(0),
      _v(0), _omega(0)
{
}

void Odometry::begin(uint32_t t0_ms)
{
    _prevTime = t0_ms;
    // prime counts
    _prevLeftCount = (_fl.read() + _rl.read()) / 2;
    _prevRightCount = (_fr.read() + _rr.read()) / 2;
}

void Odometry::update(uint32_t now_ms)
{
    uint32_t dt_ms = now_ms - _prevTime;
    if (dt_ms == 0)
        return;
    // average left & right counts
    int32_t leftCount = (_fl.read() + _rl.read()) / 2;
    int32_t rightCount = (_fr.read() + _rr.read()) / 2;
    int32_t dLeft = leftCount - _prevLeftCount;
    int32_t dRight = rightCount - _prevRightCount;

    float dL = dLeft * _metersPerTick;
    float dR = dRight * _metersPerTick;

    // robot‐frame linear and angular deltas
    float dl = (dR + dL) * 0.5f;
    float dth = (dR - dL) / _trackWidth;

    // integrate pose
    float dt = dt_ms * 0.001f;
    _x += dl * cosf(_theta + dth * 0.5f);
    _y += dl * sinf(_theta + dth * 0.5f);
    _theta += dth;

    // store velocities
    _v = dl / dt;
    _omega = dth / dt;

    // save
    _prevLeftCount = leftCount;
    _prevRightCount = rightCount;
    _prevTime = now_ms;
}
