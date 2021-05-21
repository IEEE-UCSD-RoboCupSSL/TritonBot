#include "CoreModules/DataCmdTypes.hpp"

MotionCMD defaultCmd() {
    MotionCMD dftCmd;
    dftCmd.setpoint3d = {0, 0, 0};
    dftCmd.mode = CtrlMode::TVRV;
    dftCmd.refFrame = ReferenceFrame::BodyFrame;
    return dftCmd;
}

BotData defaultBotData() {
    BotData data;
    data.pos = {0, 0};
    data.vel = {0, 0};
    data.ang = 0.0f;
    data.angVel = 0.0f;
    return data;
}

BallData defaultBallData() {
    BallData data;
    data.pos = {0, 0};
    data.vel = {0, 0};
    return data;
}