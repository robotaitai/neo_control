#pragma once
#include "MotorDriver.h"
#include "VelocityController.h"
#include "Odometry.h"
#include "CommProtocol.h"

/**
 * High‐level robot controller:
 *  - accepts (v, ω) body‐twist commands
 *  - runs four independent velocity PIDs
 *  - drives MotorDrivers
 *  - updates Odometry
 *  - sends back pose & velocity
 */
class RobotController
{
public:
    RobotController(MotorDriver &fl, MotorDriver &fr,
                    MotorDriver &rl, MotorDriver &rr,
                    VelocityController &flc,
                    VelocityController &frc,
                    VelocityController &rlc,
                    VelocityController &rrc,
                    Odometry &odom,
                    CommProtocol &comm);

    /** Call when a new <V,lin,ang> frame arrives. */
    void onNewCommand(float lin_mps, float ang_radps);

    /** Run at VEL_CTRL_HZ to update wheel PIDs & outputs. */
    void controlStep();

    /** Run at ODOM_HZ (pass millis()) to update odometry & send it. */
    void odomStep(uint32_t now_ms);

private:
    MotorDriver &_fl;
    MotorDriver &_fr;
    MotorDriver &_rl;
    MotorDriver &_rr;

    VelocityController &_flc;
    VelocityController &_frc;
    VelocityController &_rlc;
    VelocityController &_rrc;

    Odometry &_odom;
    CommProtocol &_comm;

    float _cmdLin;
    float _cmdAng;
};
