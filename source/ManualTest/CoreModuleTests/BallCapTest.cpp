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

    MotionCommand cmd = config.botConfig->autoBallCaptureSolution(isHoldingBall, ballData, botData, interpolationRate,
                                                                  0, botData.vel);

    std::cout << "cmd.setpoint3d: " << cmd.setpoint3d << std::endl;
    std::cout << "cmd.frame" << cmd.frame << std::endl;
    std::cout << "cmd.mode" << cmd.mode << std::endl;

    try {
        ManualTest::testDoubleEq("Test if cmd.setpoint3d(0) is 20", 20, cmd.setpoint3d(0), 0.1);
        ManualTest::testDoubleEq("Test if cmd.setpoint3d(1) is 10", 10, cmd.setpoint3d(1), 0.1);
        ManualTest::testDoubleEq("Test if cmd.setpoint3d(2) is within -64 and -63", -63.5, cmd.setpoint3d(2), 0.5);
        ManualTest::testModeEq("Test if cmd.mode is TDRD", TDRD, cmd.mode);
        ManualTest::testFrameEq("Test if cmd.frame is BodyFrame", BodyFrame, cmd.frame);

    } catch (std::exception &e) {
        std::cerr << e.what() << std::endl;
    }

    ballData.frame = BodyFrame;
    ballData.pos = {10, 10};
    ballData.vel = {10, 0};

    botData.frame = BodyFrame;
    botData.pos = {0, 0};
    botData.vel = {0, 0};
    botData.ang = 0;
    botData.angVel = 0;


    ManualTest::pauseAfterTest();
    return true;
}


BallCapTest::~BallCapTest() {

}
