#include "ManualTest/PeriphModuleTests/UdpReceiveModuleTest.hpp"
#include "Misc/PubSubSystem/Module.hpp"
#include <iostream>
#include <string>
#include "Misc/Utility/Systime.hpp"
#include "Config/ModuleFrequencies.hpp"
#include "PeriphModules/RemoteServers/UdpReceiveModule.hpp"
#include "Misc/Utility/Common.hpp"
#include "CoreModules/MotionModule/MotionModule.hpp"

bool UdpReceiveModuleTest::test(ThreadPool& threadPool) {
    
    std::unique_ptr<UdpReceiveModule> udpReceiveModule(new UdpReceiveModule());

    // Mock
    ITPS::FieldPublisher<arma::vec> botPosPub("From:UdpReceiveModule", "BotPos(WorldFrame)", zeroVec2d());
    ITPS::FieldPublisher<arma::vec> botVelPub("From:UdpReceiveModule", "BotVel(WorldFrame)", zeroVec2d());
    ITPS::FieldPublisher<float> botAngPub("From:UdpReceiveModule", "BotAng(WorldFrame)", 0.00);
    ITPS::FieldPublisher<float> botAngVelPub("From:UdpReceiveModule", "BotAngVel(WorldFrame)", 0.00);
    ITPS::FieldPublisher<arma::vec> ballPosPub("From:UdpReceiveModule", "BallPos(WorldFrame)", zeroVec2d());
    ITPS::FieldPublisher<arma::vec> ballVelPub("From:UdpReceiveModule", "BallVel(WorldFrame)", zeroVec2d());
    ITPS::FieldPublisher< MotionCMD > motionCmdPub("From:UdpReceiveModule", "MotionCommand", defaultCmd());
    ITPS::FieldPublisher< bool > enAutoCapPub("From:UdpReceiveModule", "EnableAutoCap", false);
    ITPS::FieldPublisher<arma::vec> kickerSetPointPub("From:UdpReceiveModule", "KickingSetPoint", zeroVec2d());

    /*** Subscriber setup ***/
    ITPS::FieldSubscriber<arma::vec> robotOriginInWorldSub("From:TcpReceiveModule", "RobotOrigin(WorldFrame)");
    ITPS::FieldSubscriber<BotData> botProcessedDataSub("MotionEKF", "BotProcessedData");
    ITPS::FieldSubscriber< MotionCMD > ballCapMotionCmdSub("From:BallCaptureModule", "MotionCommand");




    // Run the module
    udpReceiveModule->run(threadPool);

    threadPool.joinAll();
    return true;
}
