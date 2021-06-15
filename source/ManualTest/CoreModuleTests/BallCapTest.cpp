//
// Created by samuelluohaoen on 6/14/2021.
//

#include "ManualTest/CoreModuleTests/BallCapTest.hpp"

BallCapTest::BallCapTest(Config &cfg) : config(cfg) {}

bool BallCapTest::test(ThreadPool &threadPool) {

    bool isHoldingBall = false;
    BallData ballData;
    ballData.frame = BodyFrame;
    ballData.pos = {10, 10};
    ballData.vel = {10, 0};
    BotData botData;
    botData.frame = BodyFrame;
    botData.pos = {0, 0};
    botData.vel = {0, 0};
    botData.ang = 0;
    botData.angVel = 0;

    double interpolationRate = 1.0;

    MotionCommand cmd = config.botConfig->autoBallCaptureSolution(isHoldingBall, ballData, botData, interpolationRate);


    std::cout << "cmd.setpoint3d: " << cmd.setpoint3d << std::endl;
    std::cout << "cmd.frame" << cmd.frame << std::endl;
    std::cout << "cmd.mode" << cmd.mode << std::endl;

    assert(cmd.setpoint3d(0) == 20);
    assert(cmd.setpoint3d(1) == 10);
    assert((-64.0 <= cmd.setpoint3d(2)) && (cmd.setpoint3d(2) <= -63.0));
    assert(cmd.mode == TDRD);
    assert(cmd.frame == BodyFrame);

    std::cout << "All test case passed" << std::endl;
    return true;
}


BallCapTest::~BallCapTest() {

}
