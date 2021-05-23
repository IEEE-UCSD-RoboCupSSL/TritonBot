#include "CoreModules/DataCmdTypes.hpp"

MotionCommand defaultMotionCommand() {
    MotionCommand dftCmd;
    dftCmd.setpoint3d = {0, 0, 0};
    dftCmd.mode = CtrlMode::TVRV;
    dftCmd.frame = ReferenceFrame::BodyFrame;
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
    data.botData = defaultBotData();
    data.isHoldingBall = false;
    return data;
}
CameraData defaultCameraData() {
    CameraData data;
    data.ballData = defaultBallData();
    data.isBallInFov = false;
    return data;
}