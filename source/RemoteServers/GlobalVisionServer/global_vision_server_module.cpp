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

using namespace boost;

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
    ITPS::NonBlockingPublisher<arma::vec> trans_disp_pub("GVision Server", "BotPos[BodyFrame]", zero_vec_2d()); 


    logger.log(Info, "Server Started on Port Number:" + repr(GVISION_SERVER_PORT) 
                + ", Receiving Global Vision Data");

    size_t num_received;
    std::string packet_received;
    VisionData visDataReceived;
    try{
        while(true){
            num_received = socket.receive_from(boost::asio::buffer(receive_buffer), endpoint_to_listen);

            packet_received = std::string(receive_buffer.begin(), receive_buffer.begin() + num_received);
            

            //logger.log(Info, "Data received\n");
            
            visDataReceived.ParseFromString(packet_received);

            // logger.log(Info, visDataReceived.ball_pos().DebugString());

            world_data_pub.publish(visDataReceived); // publish serialized data

        }   
    }
    catch(std::exception& e)
    {
        logger.log(Error, "[Exception]" + std::string(e.what()));
    }

}