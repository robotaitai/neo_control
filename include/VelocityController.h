#pragma once
#include "PIDController.h"

/**
 * Wraps a PIDController to regulate one wheel’s linear speed (m/s).
 */
class VelocityController
{
public:
    /**
     * @param pid         A PIDController already configured with VEL_PID_* gains
     * @param wheelRadius Radius of wheel in meters
     * @param ticksPerRev Encoder ticks per full wheel revolution (quadrature included)
     */
    VelocityController(PIDController &pid,
                       float wheelRadius,
                       uint32_t ticksPerRev);

    /** Set the desired wheel speed (m/s). */
    void setTarget(float vel_mps);

    /**
     * Compute a new PWM % from the measured encoder‐derived velocity.
     * @param measured_mps measured wheel speed in m/s
     */
    void update(float measured_mps);

    /** Get output in –100…+100 (%) to feed into MotorDriver.setSpeed(). */
    int16_t getOutput() const;

private:
    PIDController &_pid;
    float _wheelRadius;
    float _metersPerTick;
    float _target;
    int16_t _outputPercent;
};
