#include "CoreModules/DataCmdTypes.hpp"

MotionCMD defaultCmd() {
    MotionCMD dftCmd;
    dftCmd.setpoint3d = {0, 0, 0};
    dftCmd.mode = CtrlMode::TVRV;
    dftCmd.refFrame = ReferenceFrame::BodyFrame;
    return dftCmd;
}