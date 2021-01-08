#include "RemoteServers/GlobalVisionServer/global_vision_server_module.hpp"

#include <string>
#include <thread>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include "Utility/systime.hpp"
#include "PubSubSystem/thread_pool.hpp"
#include "ProtoGenerated/RemoteAPI.pb.h"
#include "Utility/boost_logger.hpp"
#include "Config/config.hpp"
#include "Utility/common.hpp"
#include "EKF-Module/motion_ekf_module.hpp"
#include "EKF-Module/ball_ekf_module.hpp"

using namespace boost;


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


void GlobalVisionServer::task(ThreadPool& thread_pool)
{
    B_Log logger;
    logger.add_tag("Global Vision Server Module");
    logger(Info) << "\033[0;32m Thread Started \033[0m";

    /*** UDP server setup***/
    asio::io_service io_service;
    asio::ip::udp::endpoint endpoint_to_listen(asio::ip::udp::v4(), GVISION_SERVER_PORT);
    asio::ip::udp::socket socket(io_service, endpoint_to_listen);
    
    boost::array<char, UDP_RBUF_SIZE> receive_buffer;

    /*** Publisher setup ***/
    // Note: will convert received worldframe data to body frame in which bot position is relative to the bot origin
    ITPS::NonBlockingPublisher<arma::vec> trans_disp_pub("GVision Server", "BotPos(BodyFrame)", zero_vec_2d()); 
    ITPS::NonBlockingPublisher<arma::vec> trans_vel_pub("GVision Server", "BotVel(BodyFrame)", zero_vec_2d());
    ITPS::NonBlockingPublisher<float> rot_disp_pub("GVision Server", "BotAng(BodyFrame)", 0.00);
    ITPS::NonBlockingPublisher<float> rot_vel_pub("GVision Server", "BotAngVel(BodyFrame)", 0.00);
    ITPS::NonBlockingPublisher<arma::vec> ball_loc_pub("GVision Server", "BallPos(BodyFrame)", zero_vec_2d()); 
    ITPS::NonBlockingPublisher<arma::vec> ball_vel_pub("GVision Server", "BallVel(BodyFrame)", zero_vec_2d());

    /*** Subscriber setup ***/
    ITPS::NonBlockingSubscriber<arma::vec> robot_origin_w_sub("ConnectionInit", "RobotOrigin(WorldFrame)");
    ITPS::NonBlockingSubscriber<MotionEKF::MotionData> sensor_sub("MotionEKF", "MotionData");
    try {
        robot_origin_w_sub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        sensor_sub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
    }
    catch(std::exception& e) {
        B_Log logger;
        logger.add_tag("[global_vision_server_module.cpp]");
        logger.log(Error, e.what());
        std::exit(0);
    }

    logger.log(Info, "Server Started on Port Number:" + repr(GVISION_SERVER_PORT) 
                + ", Receiving Global Vision Data");

    size_t num_received;
    std::string packet_received;
    VisionData visDataReceived;
    arma::vec trans_disp, trans_vel, ball_loc, ball_vel;
    float rot_disp, rot_vel; 
    try{
        while(true){
            num_received = socket.receive_from(boost::asio::buffer(receive_buffer), endpoint_to_listen);

            packet_received = std::string(receive_buffer.begin(), receive_buffer.begin() + num_received);
            

            //logger.log(Info, "Data received\n");
            
            visDataReceived.ParseFromString(packet_received);

            if(false){
                logger.log(Info, "[Global Vision Server]:" + visDataReceived.DebugString());
            }

            // logger.log(Info, visDataReceived.ball_pos().DebugString());

            /*** Data type & format & ref frame conversions ***/
            // make local copies
            trans_disp = {visDataReceived.bot_pos().x(), visDataReceived.bot_pos().y()}; // not transformed yet
            trans_vel = {visDataReceived.bot_vel().x(), visDataReceived.bot_vel().y()}; // not transformed yet
            rot_disp = visDataReceived.bot_ang(); // angular data no need to transform
            rot_vel = visDataReceived.bot_ang_vel(); // angular data no need to transform
            ball_loc = {visDataReceived.ball_pos().x(), visDataReceived.ball_pos().y()}; // not transformed yet
            ball_vel = {visDataReceived.ball_vel().x(), visDataReceived.ball_vel().y()}; // not transformed yet

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

        }   
    }
    catch(std::exception& e)
    {
        logger.log(Error, "[Exception]" + std::string(e.what()));
    }

}