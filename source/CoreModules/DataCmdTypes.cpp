#include "CoreModules/DataCmdTypes.hpp"

MotionCommand defaultCmd() {
    MotionCommand dftCmd;
    dftCmd.setpoint3d = {0, 0, 0};
    dftCmd.mode = CtrlMode::TVRV;
    dftCmd.frame = ReferenceFrame::BodyFrame;
    return dftCmd;
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