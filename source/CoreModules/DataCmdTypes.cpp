#include "CoreModules/DataCmdTypes.hpp"

MotionCommand defaultMotionCommand() {
    MotionCommand dftCmd;
    dftCmd.setpoint3d = {0, 0, 0};
    dftCmd.mode = CtrlMode::TVRV;
    dftCmd.frame = ReferenceFrame::WorldFrame;
    return dftCmd;
}

Command defaultCommand() {
    Command cmd;
    cmd.enAutoCap = false;
    cmd.kickerSetPoint = {0, 0};
    cmd.motionCommand = defaultMotionCommand();
    return cmd;
}

BotData defaultBotData() {
    BotData data;
    data.pos = {0, 0};
    data.vel = {0, 0};
    data.ang = 0.0f;
    data.angVel = 0.0f;
    data.frame = NotDetermined;
    return data;
}

BallData defaultBallData() {
    BallData data;
    data.pos = {0, 0};
    data.vel = {0, 0};
    data.frame = NotDetermined;
    return data;
}


SslVisionData defaultSslVisionData() {
    SslVisionData data;
    data.botData = defaultBotData();
    data.botData.frame = ReferenceFrame::WorldFrame;
    data.ballData = defaultBallData();
    data.ballData.frame = ReferenceFrame::WorldFrame;
    return data;
}


McuSensorData defaultMcuSensorData() {
    McuSensorData data;
    data.encCnt = {0, 0};
    data.imuAcc = {0, 0};
    data.imuTheta = 0.0;
    data.imuOmega = 0.0;
    data.isHoldingBall = false;
    return data;
}

CameraData defaultCameraData() {
    CameraData data;
    data.ballData = defaultBallData();
    data.isBallInFov = false;
    return data;
}


ControlInput defaultControlInput() {
    ControlInput data;
    SetPoint<arma::vec2> transsp;
    transsp.type = SetPointType::velocity;
    transsp.value = {0.0f, 0.0f};
    SetPoint<float> rotsp;
    rotsp.type = SetPointType::velocity;
    rotsp.value = 0.0f;
    data.translationalSetPoint = transsp;
    data.rotationalSetPoint = rotsp;
    data.isNoSlowDownMode = false;
    return data;
}

PIDConstants defaultPIDConstants() {
    PIDConstants pc;
    pc.Kd = 0.00f;
    pc.Kp = 0.00f;
    pc.Ki = 0.00f;
    return pc;
}


ControlOutput defaultControlOutput() {
    ControlOutput co;
    co.vx = 0.00;
    co.vy = 0.00;
    co.omega = 0.00;
    return co;
}