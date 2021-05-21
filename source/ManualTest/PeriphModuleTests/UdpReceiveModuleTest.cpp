#include "ManualTest/PeriphModuleTests/UdpReceiveModuleTest.hpp"
#include "Misc/PubSubSystem/Module.hpp"
#include <iostream>
#include <string>
#include "Misc/Utility/Systime.hpp"
#include "Config/ModuleFrequencies.hpp"
#include "PeriphModules/RemoteServers/UdpReceiveModule.hpp"
#include "PeriphModules/RemoteServers/TcpReceiveModule.hpp"
#include "Misc/Utility/Common.hpp"
#include "CoreModules/MotionModule/MotionModule.hpp"
#include "Config/Config.hpp"

bool UdpReceiveModuleTest::test(ThreadPool& threadPool) {
    std::unique_ptr<TcpReceiveModule> tcpReceiveModule(new TcpReceiveModule()); // need to connect first
    std::unique_ptr<UdpReceiveModule> udpReceiveModule(new UdpReceiveModule());

    // Mock
    ITPS::FieldPublisher<bool> ballcapStatusPub("From:BallCaptureModule", "isDribbled", false);
    
    // Run the modules
    tcpReceiveModule->run(threadPool);
    udpReceiveModule->run(threadPool);

    // Mock
    ITPS::FieldSubscriber<arma::vec> botPosSub("From:UdpReceiveModule", "BotPos(WorldFrame)");
    // ITPS::FieldSubscriber<arma::vec> botVelSub("From:UdpReceiveModule", "BotVel(WorldFrame)");
    // ITPS::FieldSubscriber<float> botAngSub("From:UdpReceiveModule", "BotAng(WorldFrame)");
    // ITPS::FieldSubscriber<float> botAngVelSub("From:UdpReceiveModule", "BotAngVel(WorldFrame)");
    ITPS::FieldSubscriber<arma::vec> ballPosSub("From:UdpReceiveModule", "BallPos(WorldFrame)");
    // ITPS::FieldSubscriber<arma::vec> ballVelSub("From:UdpReceiveModule", "BallVel(WorldFrame)");
    // ITPS::FieldSubscriber< MotionCMD > motionCmdSub("From:UdpReceiveModule", "MotionCommand");
    // ITPS::FieldSubscriber< bool > enAutoCapSub("From:UdpReceiveModule", "EnableAutoCap");
    // ITPS::FieldSubscriber<arma::vec> kickerSetPointSub("From:UdpReceiveModule", "KickingSetPoint");

    ITPS::FieldPublisher<arma::vec> robotOriginInWorldPub("From:TcpReceiveModule", "RobotOrigin(WorldFrame)", zeroVec2d());
    ITPS::FieldPublisher<BotData> botProcessedDataPub("MotionEKF", "BotProcessedData", defaultBotData());
    ITPS::FieldPublisher< MotionCMD > ballCapMotionCmdPub("From:BallCaptureModule", "MotionCommand", defaultCmd());

    try {
        botPosSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        ballPosSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
    } catch(std::exception& e) {}

    delay(5000);

    while(true) {
        auto botpos = botPosSub.latest_msg();
        auto ballpos = ballPosSub.latest_msg();
        
        std::cout << "bot[" <<  botpos(0) << "," << botpos(1) 
                  << "] ball[" << ballpos(0) << "," << ballpos(1)  
                  << "]" << std::endl; 
        delay(std::chrono::milliseconds(10));
    }

    threadPool.joinAll();
    return true;
}
