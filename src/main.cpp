#define DEBUG_USB_COMMANDS

#include <Arduino.h>
#include "Config.h"
#include "MechConfig.h"
#include "ControlConfig.h"

#include "MotorDriver.h"
#include "Encoder.h"
#include "PIDController.h"
#include "Odometry.h"
#include "VelocityController.h"
#include "RobotController.h"
#include "CommProtocol.h"
#include "Logger.h"

//
// Instantiate hardware
//
MotorDriver flMotor(FL_PWM_PIN, FL_DIR_PIN),
    frMotor(FR_PWM_PIN, FR_DIR_PIN),
    rlMotor(RL_PWM_PIN, RL_DIR_PIN),
    rrMotor(RR_PWM_PIN, RR_DIR_PIN);

Encoder flEnc(FL_ENC_A_PIN, FL_ENC_B_PIN),
    frEnc(FR_ENC_A_PIN, FR_ENC_B_PIN),
    rlEnc(RL_ENC_A_PIN, RL_ENC_B_PIN),
    rrEnc(RR_ENC_A_PIN, RR_ENC_B_PIN);

PIDController flPid(VEL_PID_KP, VEL_PID_KI, VEL_PID_KD),
    frPid(VEL_PID_KP, VEL_PID_KI, VEL_PID_KD),
    rlPid(VEL_PID_KP, VEL_PID_KI, VEL_PID_KD),
    rrPid(VEL_PID_KP, VEL_PID_KI, VEL_PID_KD);

VelocityController flVc(flPid, WHEEL_RADIUS_M, ENCODER_TICKS_PER_REV),
    frVc(frPid, WHEEL_RADIUS_M, ENCODER_TICKS_PER_REV),
    rlVc(rlPid, WHEEL_RADIUS_M, ENCODER_TICKS_PER_REV),
    rrVc(rrPid, WHEEL_RADIUS_M, ENCODER_TICKS_PER_REV);

Odometry odom(flEnc, rlEnc, frEnc, rrEnc,
              WHEEL_RADIUS_M, TRACK_WIDTH_M,
              ENCODER_TICKS_PER_REV);

RobotController robot(flMotor, frMotor, rlMotor, rrMotor,
                      flVc, frVc, rlVc, rrVc,
                      odom,
                      /*Comm*/ *(new CommProtocol()));

void setup()
{
    Logger::begin(DBG_BAUD);
    flMotor.begin();
    frMotor.begin();
    rlMotor.begin();
    rrMotor.begin();
    flEnc.begin();
    frEnc.begin();
    rlEnc.begin();
    rrEnc.begin();

    // Serial2 for Jetson
    Serial2.begin(UP_BAUD, SERIAL_8N1, UP_RX_PIN, UP_TX_PIN);
    CommProtocol::begin(Serial2, UP_BAUD);

    #ifdef DEBUG_USB_COMMANDS
        // Run commands & telemetry over USB Serial
        Serial.begin(DBG_BAUD);
        while (!Serial)
            ; // wait for USB port open
        CommProtocol::begin(Serial, DBG_BAUD);
        Logger::info("**DEBUG MODE: commands on USB**");
    #else
        // Normal operation on UART2
        Serial2.begin(UP_BAUD, SERIAL_8N1, UP_RX_PIN, UP_TX_PIN);
        CommProtocol::begin(Serial2, UP_BAUD);
    #endif

    // prime odometry
    odom.begin(millis());
}

void loop()
{
    static uint32_t lastOdom = millis();
    static uint32_t lastCtrl = millis();

    // 1) check for new twist commands
    float lin, ang;
    if (CommProtocol::receiveTwist(lin, ang))
    {
        robot.onNewCommand(lin, ang);
    }

    // 2) odometry update
    uint32_t now = millis();
    if (now - lastOdom >= ODOM_PERIOD_MS)
    {
        lastOdom = now;
        robot.odomStep(now);
    }

    // 3) velocity control update
    if (now - lastCtrl >= VEL_CTRL_PERIOD_MS)
    {
        lastCtrl = now;
        robot.controlStep();
    }
}
