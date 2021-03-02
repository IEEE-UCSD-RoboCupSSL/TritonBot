#include "PeriphModules/RemoteServers/CmdServerModule.hpp"

#include <string>
#include <thread>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>

#include "CoreModules/MotionModule/MotionModule.hpp"
#include "Misc/Utility/Systime.hpp"
#include "Misc/Utility/BoostLogger.hpp"
#include "Misc/Utility/Common.hpp"
#include "Misc/PubSubSystem/ThreadPool.hpp"
#include "Config/Config.hpp"
#include "ProtoGenerated/RemoteAPI.pb.h"


using namespace boost;
using namespace boost::asio;
using namespace boost::asio::ip;


static Motion::MotionCMD default_cmd() {
    Motion::MotionCMD dft_cmd;
    dft_cmd.setpoint_3d = {0, 0, 0};
    dft_cmd.mode = Motion::CTRL_Mode::TVRV;
    dft_cmd.ref_frame = Motion::ReferenceFrame::BodyFrame;
    return dft_cmd;
}

// Implementation of task to be run on this thread
[[noreturn]] void CMDServer::task(ThreadPool& thread_pool) {
    UNUSED(thread_pool); 

    B_Log logger;
    logger.add_tag("Command Server Module");
    logger(Info) << "\033[0;32m Thread Started \033[0m";

    io_service io_service;
    udp::endpoint ep_listen(udp::v4(), CMD_SERVER_PORT);
    udp::socket socket(io_service, ep_listen);

    size_t num_received;
    std::string packet_received;
    boost::array<char, UDP_RBUF_SIZE> receive_buffer;



    ITPS::NonBlockingPublisher< Motion::MotionCMD > m_cmd_pub("CMD Server", "MotionCMD", default_cmd());
    
    ITPS::NonBlockingSubscriber< Motion::MotionCMD > capture_cmd_sub("Ball Capture Module", "MotionCMD");

    ITPS::NonBlockingPublisher< bool > en_autocap_pub("CMD Server", "EnableAutoCap", false);

    ITPS::NonBlockingPublisher<arma::vec> kicker_pub("Kicker", "KickingSetPoint", zero_vec_2d());

    try {
        capture_cmd_sub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
    }
    catch(std::exception& e) {
        B_Log logger;
        logger.add_tag("[cmd_server_module.cpp]");
        logger.log(Error, std::string(e.what()));
        std::exit(0);
    }

    logger.log(Info, "CMD Server Started on Port Number:" + repr(CMD_SERVER_PORT) 
                + ", Listening to Remote AI Commands... ");

    Commands cmd;
    Motion::MotionCMD m_cmd;
    arma::vec kick_vec2d = {0, 0};

    while(1) { // has delay (good for reducing high CPU usage)
        num_received = socket.receive_from(asio::buffer(receive_buffer), ep_listen);
        packet_received = std::string(receive_buffer.begin(), receive_buffer.begin() + num_received);
        // logger.log(Info, packet_received);
        cmd.ParseFromString(packet_received);

        // logger.log(Debug, cmd.DebugString());

        if(cmd.enable_ball_auto_capture() == false) {
            // Listening to remote motion commands
            en_autocap_pub.publish(false);
            switch((int)cmd.mode()) {
                case 0: m_cmd.mode = Motion::CTRL_Mode::TDRD; break;
                case 1: m_cmd.mode = Motion::CTRL_Mode::TDRV; break;
                case 2: m_cmd.mode = Motion::CTRL_Mode::TVRD; break;
                case 3: m_cmd.mode = Motion::CTRL_Mode::TVRV; break;
                case 4: m_cmd.mode = Motion::CTRL_Mode::NSTDRD; break;
                case 5: m_cmd.mode = Motion::CTRL_Mode::NSTDRV; break;
                default: m_cmd.mode = Motion::CTRL_Mode::TVRV;
            }
            if(cmd.is_world_frame()) {
                m_cmd.ref_frame = Motion::WorldFrame;
            }
            else {
                m_cmd.ref_frame = Motion::BodyFrame;
            }

            m_cmd.setpoint_3d = {cmd.motion_set_point().x(), 
                                 cmd.motion_set_point().y(),
                                 cmd.motion_set_point().z()};
            m_cmd_pub.publish(m_cmd);

            kick_vec2d = {cmd.kicker_set_point().x(), cmd.kicker_set_point().y()};
            kicker_pub.publish(kick_vec2d);
            

        }
        else {
            // Listening to internal AutoCapture module's commands
            en_autocap_pub.publish(true);
            m_cmd = capture_cmd_sub.latest_msg();
            m_cmd_pub.publish(m_cmd);

            // kick_vec2d = {cmd.kicker_set_point().x(), cmd.kicker_set_point().y()};
            // kicker_pub.publish(kick_vec2d);

        }

        delay(1);

    }

    io_service.run();
    
}