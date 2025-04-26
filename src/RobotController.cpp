#include "RobotController.h"
#include "ControlConfig.h"

RobotController::RobotController(MotorDriver &fl, MotorDriver &fr,
                                 MotorDriver &rl, MotorDriver &rr,
                                 VelocityController &flc, VelocityController &frc,
                                 VelocityController &rlc, VelocityController &rrc,
                                 Odometry &odom,
                                 CommProtocol &comm)
    : _fl(fl), _fr(fr), _rl(rl), _rr(rr),
      _flc(flc), _frc(frc), _rlc(rlc), _rrc(rrc),
      _odom(odom), _comm(comm),
      _cmdLin(0), _cmdAng(0)
{
}

void RobotController::onNewCommand(float lin_mps, float ang_radps)
{
    _cmdLin = lin_mps;
    _cmdAng = ang_radps;
}

void RobotController::controlStep()
{
    // compute wheel targets for differential drive
    float halfTrack = TRACK_WIDTH_M * 0.5f;
    float vL = _cmdLin - _cmdAng * halfTrack;
    float vR = _cmdLin + _cmdAng * halfTrack;

    // update each wheel’s PID
    _flc.setTarget(vL);
    _flc.update(_odom.linVel());
    _rlc.setTarget(vL);
    _rlc.update(_odom.linVel());
    _frc.setTarget(vR);
    _frc.update(_odom.linVel());
    _rrc.setTarget(vR);
    _rrc.update(_odom.linVel());

    // drive motors
    _fl.setSpeed(_flc.getOutput());
    _rl.setSpeed(_rlc.getOutput());
    _fr.setSpeed(_frc.getOutput());
    _rr.setSpeed(_rrc.getOutput());
}

void RobotController::odomStep(uint32_t now_ms)
{
    _odom.update(now_ms);
    // send back pose and velocities
    struct
    {
        float x, y, theta, v, omega;
    } s = {
        _odom.x(),
        _odom.y(),
        _odom.theta(),
        _odom.linVel(),
        _odom.angVel()};
    _comm.sendOdometry(s.x, s.y, s.theta, s.v, s.omega);
}
