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
    std::unique_ptr<UdpReceiveModule> udpReceiveModule(new UdpReceiveModule());
    
    // Run the modules
    udpReceiveModule->run(threadPool);

    // Mock
    ITPS::FieldSubscriber< MotionCommand > motionCmdSub("From:UdpReceiveModule", "MotionCommand");
    ITPS::FieldSubscriber< bool > enAutoCapSub("From:UdpReceiveModule", "EnableAutoCap");
    ITPS::FieldSubscriber<arma::vec> kickerSetPointSub("From:UdpReceiveModule", "KickingSetPoint");


    ITPS::FieldSubscriber<BotData> receivedBotDataSub("From:UdpReceiveModule", "BotData(WorldFrame)");
    ITPS::FieldSubscriber<BallData> receivedBallDataSub("From:UdpReceiveModule", "BallData(WorldFrame)");


    try {
        motionCmdSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        enAutoCapSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        kickerSetPointSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);

        receivedBallDataSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        receivedBotDataSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
    } catch(std::exception& e) {}

    delay(std::chrono::milliseconds(1500));

    int modeIdx = 0;
    std::cout << "Enter 0 to print data: bot-pos | bot-vel | bot-ang | bot-angvel" << std::endl;
    std::cout << "Enter 1 to print data: ball-pos | ball-vel" << std::endl;
    std::cout << "Enter 2 to print cmd: motion-cmd" << std::endl;
    std::cout << "Enter 3 to print cmd: en-auto-cap | kicker-setpoint" << std::endl;
    std::cout << ">> " << std::flush;
    scanf("%d", &modeIdx);

    while(true) {
        
        if(modeIdx == 0) {
            auto botpos = receivedBotDataSub.latest_msg().pos;
            auto botvel = receivedBotDataSub.latest_msg().vel;    
            std::cout << "pos[" <<  botpos(0) << "," << botpos(1) 
                    << "] vel[" << botvel(0) << "," << botvel(1)  
                    << "]" << " ang[" << receivedBotDataSub.latest_msg().ang
                    << "]" << " angvel[" << receivedBotDataSub.latest_msg().angVel << "]"
                    << std::endl;
        } 

        if(modeIdx == 1) {
            auto ballpos = receivedBallDataSub.latest_msg().pos;
            auto ballvel = receivedBallDataSub.latest_msg().vel;    
            std::cout << "pos[" <<  ballpos(0) << "," << ballpos(1) 
                    << "] vel[" << ballvel(0) << "," << ballvel(1)  
                    << "]" << std::endl;
        } 

        if(modeIdx == 2) {
            auto mcmd = motionCmdSub.latest_msg();
            std::string mode;
            if(mcmd.mode == CtrlMode::TDRD) mode = "TDRD";
            if(mcmd.mode == CtrlMode::TDRV) mode = "TDRV";
            if(mcmd.mode == CtrlMode::TVRD) mode = "TVRD";
            if(mcmd.mode == CtrlMode::TVRV) mode = "TVRV";
            if(mcmd.mode == CtrlMode::NSTDRD) mode = "NSTDRD";
            if(mcmd.mode == CtrlMode::NSTDRV) mode = "NSTDRV";
            std::cout << "setpoint3d[" <<  mcmd.setpoint3d(0) << ","
                    << mcmd.setpoint3d(1) << "," << mcmd.setpoint3d(2) 
                    << "] frame[" << (mcmd.frame == ReferenceFrame::WorldFrame 
                        ? "world" : "body")  << "] mode[" << mode << "]" << std::endl;
        }

        if(modeIdx == 3) {
            auto kicksp = kickerSetPointSub.latest_msg();
            std::cout << "autocap[" << (enAutoCapSub.latest_msg() ? "true" : "false") 
                      << "] kick-setpoint[" << kicksp(0) << "," << kicksp(1) << "]" << std::endl;
        }
        delay(std::chrono::milliseconds(10));
    }

    threadPool.joinAll();
    return true;
}
