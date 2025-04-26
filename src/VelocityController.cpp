#include "VelocityController.h"
#include <algorithm>

VelocityController::VelocityController(PIDController &pid,
                                       float wheelRadius,
                                       uint32_t ticksPerRev)
    : _pid(pid),
      _wheelRadius(wheelRadius),
      _metersPerTick((2.0f * wheelRadius) * (float)M_PI / ticksPerRev),
      _target(0),
      _outputPercent(0)
{
}

void VelocityController::setTarget(float vel_mps)
{
    _target = vel_mps;
}

void VelocityController::update(float measured_mps)
{
    float u = _pid.update(_target, measured_mps); // –100…+100
    // clamp and store
    if (u > 100)
        u = 100;
    if (u < -100)
        u = -100;
    _outputPercent = (int16_t)u;
}

int16_t VelocityController::getOutput() const
{
    return _outputPercent;
}
