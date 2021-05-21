#include "PeriphModules/RemoteServers/UdpReceiveModule.hpp"

#include <string>
#include <thread>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>

#include "CoreModules/MotionModule/MotionModule.hpp"
#include "CoreModules/EKF-Module/MotionEkfModule.hpp"
#include "CoreModules/EKF-Module/BallEkfModule.hpp"
#include "Misc/Utility/Systime.hpp"
#include "Misc/Utility/BoostLogger.hpp"
#include "Misc/Utility/Common.hpp"
#include "Misc/PubSubSystem/ThreadPool.hpp"
#include "Config/Config.hpp"
#include "ProtoGenerated/RemoteAPI.pb.h"
#include "Config/ModuleFrequencies.hpp"

using namespace boost;
using namespace boost::asio;
using namespace boost::asio::ip;

static arma::vec transform(arma::vec, float, arma::vec);

// Implementation of task to be run on this thread
void UdpReceiveModule::task(ThreadPool& threadPool) {
    UNUSED(threadPool); 

    BLogger logger;
    logger.addTag("UDP Receiver Module");
    logger(Info) << "\033[0;32m Thread Started \033[0m";

    io_service ios;
    udp::endpoint ep(udp::v4(), UDP_PORT);
    udp::socket socket(ios, ep);

    size_t numReceived;
    std::string packetReceived;
    boost::array<char, UDP_RBUF_SIZE> receiveBuffer;


    /*** Publisher Setup ***/
    ITPS::FieldPublisher<arma::vec> botPosPub("From:UdpReceiveModule", "BotPos(WorldFrame)", zeroVec2d());
    ITPS::FieldPublisher<arma::vec> botVelPub("From:UdpReceiveModule", "BotVel(WorldFrame)", zeroVec2d());
    ITPS::FieldPublisher<float> botAngPub("From:UdpReceiveModule", "BotAng(WorldFrame)", 0.00);
    ITPS::FieldPublisher<float> botAngVelPub("From:UdpReceiveModule", "BotAngVel(WorldFrame)", 0.00);
    ITPS::FieldPublisher<arma::vec> ballPosPub("From:UdpReceiveModule", "BallPos(WorldFrame)", zeroVec2d());
    ITPS::FieldPublisher<arma::vec> ballVelPub("From:UdpReceiveModule", "BallVel(WorldFrame)", zeroVec2d());
    ITPS::FieldPublisher< MotionCommand > motionCmdPub("From:UdpReceiveModule", "MotionCommand", defaultCmd());
    ITPS::FieldPublisher< bool > enAutoCapPub("From:UdpReceiveModule", "EnableAutoCap", false);
    ITPS::FieldPublisher<arma::vec> kickerSetPointPub("From:UdpReceiveModule", "KickingSetPoint", zeroVec2d());




    logger.log(Info, "UDP Receiver Started on Port Number:" + repr(UDP_PORT)
                + ", Listening to Remote AI Commands... ");

    UDPData udpData;


    MotionCommand mCmd;
    arma::vec kickVec2d = {0, 0};
    arma::vec botPos, botVel, ballPos, ballVel;
    float botAng, botAngVel;

    while(true) {
        periodic_session([&](){
            numReceived = socket.receive_from(asio::buffer(receiveBuffer), ep);
            packetReceived = std::string(receiveBuffer.begin(), receiveBuffer.begin() + numReceived);
            // logger.log(Info, packetReceived);
            udpData.ParseFromString(packetReceived);

            // logger.log(Debug, udpData.commanddata().DebugString());

            botPos = {udpData.visiondata().bot_pos().x(), udpData.visiondata().bot_pos().y()}; // not transformed yet
            botVel = {udpData.visiondata().bot_vel().x(), udpData.visiondata().bot_vel().y()}; // not transformed yet
            botAng = udpData.visiondata().bot_ang(); // angular data no need to transform
            botAngVel = udpData.visiondata().bot_ang_vel(); // angular data no need to transform
            ballPos = {udpData.visiondata().ball_pos().x(), udpData.visiondata().ball_pos().y()}; // not transformed yet
            ballVel = {udpData.visiondata().ball_vel().x(), udpData.visiondata().ball_vel().y()}; // not transformed yet

            // reference frame transformation math
            arma::vec botOrigin = robotOriginInWorldSub.latest_msg();
            float botOrien = botDataSub.latest_msg().ang;
            botPos = transform(botOrigin, botOrien, botPos);
            botVel = transform(botOrigin, botOrien, botVel);
            ballPos = transform(botOrigin, botOrien, ballPos);
            ballVel = transform(botOrigin, botOrien, ballVel);

            // These are all body frames
            botPosPub.publish(botPos);
            botVelPub.publish(botVel);
            botAngPub.publish(botAng);
            botAngVelPub.publish(botAngVel);
            ballPosPub.publish(ballPos);
            ballVelPub.publish(ballVel);

            if(!udpData.commanddata().enable_ball_auto_capture()) {
                // Listening to remote motion commands
                enAutoCapPub.publish(false);
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
                    mCmd.refFrame = ReferenceFrame::WorldFrame;
                }
                else {
                    mCmd.refFrame = ReferenceFrame::BodyFrame;
                }
                mCmd.setpoint3d = {udpData.commanddata().motion_set_point().x(),
                                    udpData.commanddata().motion_set_point().y(),
                                    udpData.commanddata().motion_set_point().z()};
                motionCmdPub.publish(mCmd);

                kickVec2d = {udpData.commanddata().kicker_set_point().x(), udpData.commanddata().kicker_set_point().y()};
                kickerSetPointPub.publish(kickVec2d);
            }
            else {
                // Listening to internal AutoCapture module's commands
                enAutoCapPub.publish(true);
                mCmd = ballCapMotionCmdSub.latest_msg();
                motionCmdPub.publish(mCmd);
            }
        }, TO_PERIOD(UDP_RECEIVE_FREQUENCY));
    }
}

// for explaination of the math, check motion_module.cpp
static arma::vec transform(arma::vec origin, float orien, arma::vec point2d) {
    arma::mat A = wtb_homo_transform(origin, orien); // world to body homogeneous transformation
    arma::vec p_homo_w = {point2d(0), point2d(1), 1}; // homogeneous point end with a 1 (vector end with a 0)
    arma::vec p_homo_b = A * p_homo_w; // apply transformation to get the same point represented in the body frame
    // if division factor is approx. eq to zero
    if(std::fabs(p_homo_b(2)) < 0.000001) {
        p_homo_b(2) = 0.000001;
    }
    // update setpoint to the setpoint in robot's perspective (cartesean coordinate)
    arma::vec p_cart_b = {p_homo_b(0)/p_homo_b(2), p_homo_b(1)/p_homo_b(2)}; // the division is to divide the scaling factor, according to rules of homogeneous coord systems
    return p_cart_b;
}

