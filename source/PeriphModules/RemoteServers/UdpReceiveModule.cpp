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

using namespace boost;
using namespace boost::asio;
using namespace boost::asio::ip;

static arma::vec transform(arma::vec, float, arma::vec);

static Motion::MotionCMD default_cmd() {
    Motion::MotionCMD dft_cmd;
    dft_cmd.setpoint_3d = {0, 0, 0};
    dft_cmd.mode = Motion::CTRL_Mode::TVRV;
    dft_cmd.ref_frame = Motion::ReferenceFrame::BodyFrame;
    return dft_cmd;
}

// Implementation of task to be run on this thread
[[noreturn]] void CMDServer::task(ThreadPool& threadPool) {
    UNUSED(threadPool); 

    B_Log logger;
    logger.add_tag("UDP Receiver Module");
    logger(Info) << "\033[0;32m Thread Started \033[0m";

    io_service io_service;
    udp::endpoint ep_listen(udp::v4(), UDP_PORT);
    udp::socket socket(io_service, ep_listen);

    size_t num_received;
    std::string packet_received;
    boost::array<char, UDP_RBUF_SIZE> receive_buffer;


    /*** Publisher Setup ***/

    // Note: will convert received worldframe data to body frame in which bot position is relative to the bot origin
    ITPS::NonBlockingPublisher<arma::vec> trans_disp_pub("GVision Server", "BotPos(BodyFrame)", zero_vec_2d());
    ITPS::NonBlockingPublisher<arma::vec> trans_vel_pub("GVision Server", "BotVel(BodyFrame)", zero_vec_2d());
    ITPS::NonBlockingPublisher<float> rot_disp_pub("GVision Server", "BotAng(BodyFrame)", 0.00);
    ITPS::NonBlockingPublisher<float> rot_vel_pub("GVision Server", "BotAngVel(BodyFrame)", 0.00);
    ITPS::NonBlockingPublisher<arma::vec> ball_loc_pub("GVision Server", "BallPos(BodyFrame)", zero_vec_2d());
    ITPS::NonBlockingPublisher<arma::vec> ball_vel_pub("GVision Server", "BallVel(BodyFrame)", zero_vec_2d());
    ITPS::NonBlockingPublisher< Motion::MotionCMD > m_cmd_pub("CMD Server", "MotionCMD", default_cmd());
    ITPS::NonBlockingPublisher< bool > en_autocap_pub("CMD Server", "EnableAutoCap", false);
    ITPS::NonBlockingPublisher<arma::vec> kicker_pub("Kicker", "KickingSetPoint", zero_vec_2d());

    /*** Subscriber setup ***/
    ITPS::NonBlockingSubscriber<arma::vec> robot_origin_w_sub("ConnectionInit", "RobotOrigin(WorldFrame)");
    ITPS::NonBlockingSubscriber<MotionEKF::MotionData> sensor_sub("MotionEKF", "MotionData");
    ITPS::NonBlockingSubscriber< Motion::MotionCMD > capture_cmd_sub("Ball Capture Module", "MotionCMD");

    try {
        capture_cmd_sub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        robot_origin_w_sub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        sensor_sub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
    }
    catch(std::exception& e) {
        B_Log logger;
        logger.add_tag("[UdpReceiveModule.cpp]");
        logger.log(Error, std::string(e.what()));
        std::exit(0);
    }

    logger.log(Info, "UDP Receiver Started on Port Number:" + repr(UDP_PORT)
                + ", Listening to Remote AI Commands... ");

    UDPData udpData;


    Motion::MotionCMD m_cmd;
    arma::vec kick_vec2d = {0, 0};
    arma::vec trans_disp, trans_vel, ball_loc, ball_vel;
    float rot_disp, rot_vel;

    while(1) { // has delay (good for reducing high CPU usage)
        num_received = socket.receive_from(asio::buffer(receive_buffer), ep_listen);
        packet_received = std::string(receive_buffer.begin(), receive_buffer.begin() + num_received);
        // logger.log(Info, packet_received);
        udpData.ParseFromString(packet_received);

        // logger.log(Debug, udpData.commanddata().DebugString());

        trans_disp = {udpData.visiondata().bot_pos().x(), udpData.visiondata().bot_pos().y()}; // not transformed yet
        trans_vel = {udpData.visiondata().bot_vel().x(), udpData.visiondata().bot_vel().y()}; // not transformed yet
        rot_disp = udpData.visiondata().bot_ang(); // angular data no need to transform
        rot_vel = udpData.visiondata().bot_ang_vel(); // angular data no need to transform
        ball_loc = {udpData.visiondata().ball_pos().x(), udpData.visiondata().ball_pos().y()}; // not transformed yet
        ball_vel = {udpData.visiondata().ball_vel().x(), udpData.visiondata().ball_vel().y()}; // not transformed yet

        // reference frame transformation math
        arma::vec bot_origin = robot_origin_w_sub.latest_msg();
        float bot_orien = sensor_sub.latest_msg().rotat_disp;
        trans_disp = transform(bot_origin, bot_orien, trans_disp);
        trans_vel = transform(bot_origin, bot_orien, trans_vel);
        ball_loc = transform(bot_origin, bot_orien, ball_loc);
        ball_vel = transform(bot_origin, bot_orien, ball_vel);

        // These are all body frames
        trans_disp_pub.publish(trans_disp);
        trans_vel_pub.publish(trans_vel);
        rot_disp_pub.publish(rot_disp);
        rot_vel_pub.publish(rot_vel);
        ball_loc_pub.publish(ball_loc);
        ball_vel_pub.publish(ball_vel);

        if(!udpData.commanddata().enable_ball_auto_capture()) {
            // Listening to remote motion commands
            en_autocap_pub.publish(false);
            switch((int)udpData.commanddata().mode()) {
                case 0: m_cmd.mode = Motion::CTRL_Mode::TDRD; break;
                case 1: m_cmd.mode = Motion::CTRL_Mode::TDRV; break;
                case 2: m_cmd.mode = Motion::CTRL_Mode::TVRD; break;
                case 3: m_cmd.mode = Motion::CTRL_Mode::TVRV; break;
                case 4: m_cmd.mode = Motion::CTRL_Mode::NSTDRD; break;
                case 5: m_cmd.mode = Motion::CTRL_Mode::NSTDRV; break;
                default: m_cmd.mode = Motion::CTRL_Mode::TVRV;
            }
            if(udpData.commanddata().is_world_frame()) {
                m_cmd.ref_frame = Motion::WorldFrame;
            }
            else {
                m_cmd.ref_frame = Motion::BodyFrame;
            }
            m_cmd.setpoint_3d = {udpData.commanddata().motion_set_point().x(),
                                 udpData.commanddata().motion_set_point().y(),
                                 udpData.commanddata().motion_set_point().z()};
            m_cmd_pub.publish(m_cmd);

            kick_vec2d = {udpData.commanddata().kicker_set_point().x(), udpData.commanddata().kicker_set_point().y()};
            kicker_pub.publish(kick_vec2d);


        }
        else {
            // Listening to internal AutoCapture module's commands
            en_autocap_pub.publish(true);
            m_cmd = capture_cmd_sub.latest_msg();
            m_cmd_pub.publish(m_cmd);

            // kick_vec2d = {udpData.commanddata().kicker_set_point().x(), udpData.commanddata().kicker_set_point().y()};
            // kicker_pub.publish(kick_vec2d);

        }

        delay(1);

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

