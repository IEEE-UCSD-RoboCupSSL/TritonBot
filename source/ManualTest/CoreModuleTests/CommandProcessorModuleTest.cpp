#include "ManualTest/CoreModuleTests/CommandProcessorModuleTest.hpp"
#include "CoreModules/DataCmdTypes.hpp"
#include "DataFusion/BallDataFusion.hpp"
#include "DataFusion/BotDataFusion.hpp"
#include "CoreModules/CommandProcessorModule/CommandProcessorModule.hpp"
#include "Misc/Utility/Common.hpp"
#include "Misc/Utility/ClockUtil.hpp"
#include "Misc/Utility/BoostLogger.hpp"
#include "Config/Config.hpp"
#include <cassert>

bool CommandProcessorModuleTest::test(ThreadPool& threadPool) {
   
    CommandProcessorModule CmdProcModule(config);
    CmdProcModule.run(threadPool);



    /*** Mock Subscribers setup ***/
    ITPS::FieldSubscriber<bool> dribblerCommandSub("From:CommandProcessorModule", "dribblerSwitch");
    ITPS::FieldSubscriber<arma::vec2> kickerSetPointSub("From:CommandProcessorModule", "KickerSetPoint(On/Off)");
    ITPS::FieldSubscriber<bool> enableAutoCaptureSub("From:CommandProcessorModule", "EnableAutoCapture");
    ITPS::FieldSubscriber<ControlInput> controlInputSub("From:CommandProcessorModule", "MotionControlInput");

    /*** Mock Publishers setup ***/
    ITPS::FieldPublisher<Command> receivedCommandPub("From:UdpReceiveModule", "Command", defaultCommand());
    ITPS::FieldPublisher<arma::vec2> robotOriginInWorldPub("From:TcpReceiveModule", "RobotOrigin(WorldFrame)", zeroVec2d());
    ITPS::FieldPublisher<BotData> filteredBotDataPub("From:DataProcessorModule", "BotData(BodyFrame)", defaultBotData());
    ITPS::FieldPublisher<BallData> filteredBallDataPub("From:DataProcessorModule", "BallData(BodyFrame)", defaultBallData());
    ITPS::FieldPublisher<MotionCommand> ballAutoCapMotionCommandPub("From:BallAutoCaptureModule", "MotionCommand(BodyFrame)", defaultMotionCommand());

    try {
        dribblerCommandSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        kickerSetPointSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        enableAutoCaptureSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        controlInputSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
    }
    catch(std::exception& e) {
        BLogger logger;
        logger.addTag("[CommandProcessorModuleTest.cpp]");
        logger.log(Error, std::string(e.what()));
        std::exit(0);
    }

    BallData bd = defaultBallData();
    BotData btd = defaultBotData();
    bd.frame = ReferenceFrame::BodyFrame;
    btd.frame = ReferenceFrame::BodyFrame;
    filteredBallDataPub.publish(bd);
    filteredBotDataPub.publish(btd);
    delay(10);
    


    Command cmdMock;
    cmdMock.enAutoCap = true;
    cmdMock.kickerSetPoint = {1.23, 4.56};
    cmdMock.motionCommand = defaultMotionCommand();
    receivedCommandPub.publish(cmdMock);
    bd.pos = {0, 150};
    filteredBallDataPub.publish(bd);
    delay(10);
    assert(enableAutoCaptureSub.getMsg() == true);
    assert(arma::approx_equal(kickerSetPointSub.getMsg(),
                             cmdMock.kickerSetPoint, "absdiff", 0.001) 
    );
    assert(dribblerCommandSub.getMsg() == true);


    MotionCommand mCmd;
    mCmd.frame = ReferenceFrame::WorldFrame;
    mCmd.mode = CtrlMode::TDRD;
    mCmd.setpoint3d = {1000, 2000, 90};
    cmdMock.motionCommand = mCmd;
    receivedCommandPub.publish(cmdMock);
    delay(10);
    std::cout << "[A]: " << controlInputSub.getMsg().translationalSetPoint.value << std::endl;
    assert(arma::approx_equal(controlInputSub.getMsg().translationalSetPoint.value, 
                    zeroVec2d(), "absdiff", 0.001)
    );

    
    cmdMock.enAutoCap = false;
    receivedCommandPub.publish(cmdMock);
    delay(10);
    assert(enableAutoCaptureSub.getMsg() == false);
    assert(dribblerCommandSub.getMsg() == true);
    std::cout << "[B]: "<< controlInputSub.getMsg().translationalSetPoint.value << std::endl;
    arma::vec2 correctResult = {1000, 2000};
    assert(arma::approx_equal(controlInputSub.getMsg().translationalSetPoint.value, 
                    correctResult, "absdiff", 0.001)
    );



    btd.ang = 90;
    filteredBotDataPub.publish(btd);
    delay(10);
    std::cout << "[C]: " << controlInputSub.getMsg().translationalSetPoint.value << std::endl;
    correctResult = {2000, -1000};
    assert(arma::approx_equal(controlInputSub.getMsg().translationalSetPoint.value, 
                    correctResult, "absdiff", 0.001)
    );

    mCmd.mode = CtrlMode::TVRD;
    cmdMock.motionCommand = mCmd;
    receivedCommandPub.publish(cmdMock);
    delay(10);
    std::cout << "[C]: " << controlInputSub.getMsg().translationalSetPoint.value << std::endl;
    correctResult = {2000, -1000};
    assert(arma::approx_equal(controlInputSub.getMsg().translationalSetPoint.value, 
                    correctResult, "absdiff", 0.001)
    );


    arma::vec2 origin = {-123, -123};
    robotOriginInWorldPub.publish(origin);
    delay(10);
    std::cout << "[D]: " << controlInputSub.getMsg().translationalSetPoint.value << std::endl;
    correctResult = {2000, -1000};
    assert(arma::approx_equal(controlInputSub.getMsg().translationalSetPoint.value, 
                    correctResult, "absdiff", 0.001)
    );

    mCmd.mode = CtrlMode::TDRD;
    cmdMock.motionCommand = mCmd;
    receivedCommandPub.publish(cmdMock);
    delay(10);
    std::cout << "[E]: " << controlInputSub.getMsg().translationalSetPoint.value << std::endl;
    correctResult = {2123, -1123};
    assert(arma::approx_equal(controlInputSub.getMsg().translationalSetPoint.value, 
                    correctResult, "absdiff", 0.001)
    );


    std::cout << "All test case passed" << std::endl;
    threadPool.joinAll();
    return true;
}