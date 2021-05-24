#include "PeriphModules/RemoteServers/UdpReceiveModule.hpp"

#include <string>
#include <thread>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>

#include "CoreModules/MotionModule/MotionModule.hpp"
#include "CoreModules/EKF-Module/MotionEkfModule.hpp"
#include "CoreModules/EKF-Module/BallEkfModule.hpp"
#include "Misc/Utility/ClockUtil.hpp"
#include "Misc/Utility/BoostLogger.hpp"
#include "Misc/Utility/Common.hpp"
#include "Misc/PubSubSystem/ThreadPool.hpp"
#include "Config/Config.hpp"
#include "ProtoGenerated/RemoteAPI.pb.h"
#include "Config/ModuleFrequencies.hpp"
#include "CoreModules/DataCmdTypes.hpp"

using namespace boost;
using namespace boost::asio;
using namespace boost::asio::ip;


// Implementation of task to be run on this thread
void UdpReceiveModule::task(ThreadPool& threadPool) {
    UNUSED(threadPool); 

    BLogger logger;
    logger.addTag("UDP Receiver Module");
    logger(Info) << "\033[0;32m Thread Started \033[0m";

    io_service ios;
    udp::endpoint ep(udp::v4(), config.cliConfig.udpPort);
    udp::socket socket(ios, ep);

    size_t numReceived;
    std::string packetReceived;
    boost::array<char, UDP_RBUF_SIZE> receiveBuffer;


    /*** Publisher Setup ***/
    ITPS::FieldPublisher<Command> receivedCommandPub("From:UdpReceiveModule", "Command", defaultCommand());
    ITPS::FieldPublisher<SslVisionData> receivedSslVisionDataPub("From:UdpReceiveModule", "SslVision:BotData&BallData(WorldFrame)", defaultSslVisionData());
    

    logger.log(Info, "UDP Receiver Started on Port Number:" + repr(config.cliConfig.udpPort)
                + ", Listening to Remote AI Commands... ");

    UDPData udpData;


    BotData botData;
    BallData ballData;
    MotionCommand mCmd;
    arma::vec2 kickVec2d = {0, 0};

    Command cmd;
    SslVisionData data;


    while(true) {
        periodic_session([&](){
            numReceived = socket.receive_from(asio::buffer(receiveBuffer), ep);
            packetReceived = std::string(receiveBuffer.begin(), receiveBuffer.begin() + numReceived);
            // logger.log(Info, packetReceived);
            udpData.ParseFromString(packetReceived);

            // logger.log(Debug, udpData.commanddata().DebugString());
            
            /* pack received data */
            botData.pos = {udpData.visiondata().bot_pos().x(), udpData.visiondata().bot_pos().y()}; // not transformed yet
            botData.vel = {udpData.visiondata().bot_vel().x(), udpData.visiondata().bot_vel().y()}; // not transformed yet
            botData.ang = udpData.visiondata().bot_ang(); // angular data no need to transform
            botData.angVel = udpData.visiondata().bot_ang_vel(); // angular data no need to transform
            botData.frame = ReferenceFrame::WorldFrame;

            ballData.pos = {udpData.visiondata().ball_pos().x(), udpData.visiondata().ball_pos().y()}; // not transformed yet
            ballData.vel = {udpData.visiondata().ball_vel().x(), udpData.visiondata().ball_vel().y()}; // not transformed yet
            ballData.frame = ReferenceFrame::WorldFrame;

            data.botData = botData;
            data.ballData = ballData;


            /* pack received commands */
            cmd.enAutoCap = udpData.commanddata().enable_ball_auto_capture();
            switch((int)udpData.commanddata().mode()) {
                case 0: mCmd.mode = CtrlMode::TDRD; break;
                case 1: mCmd.mode = CtrlMode::TDRV; break;
                case 2: mCmd.mode = CtrlMode::TVRD; break;
                case 3: mCmd.mode = CtrlMode::TVRV; break;
                case 4: mCmd.mode = CtrlMode::NSTDRD; break;
                case 5: mCmd.mode = CtrlMode::NSTDRV; break;
                default: mCmd.mode = CtrlMode::TVRV;
            }
            if(udpData.commanddata().is_world_frame()) {
                mCmd.frame = ReferenceFrame::WorldFrame;
            }
            else {
                mCmd.frame = ReferenceFrame::BodyFrame;
            }
            mCmd.setpoint3d = {udpData.commanddata().motion_set_point().x(),
                                udpData.commanddata().motion_set_point().y(),
                                udpData.commanddata().motion_set_point().z()};
            cmd.motionCommand = mCmd;

            kickVec2d = {udpData.commanddata().kicker_set_point().x(), udpData.commanddata().kicker_set_point().y()};
            cmd.kickerSetPoint = kickVec2d;

            /* publish */
            receivedSslVisionDataPub.publish(data);
            receivedCommandPub.publish(cmd);
 
        }, TO_PERIOD(UDP_RECEIVE_FREQUENCY));
    }
}


