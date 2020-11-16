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

    VisionData vis_data;

    /*** UDP server setup***/
    asio::io_service io_service;
    asio::ip::udp::endpoint endpoint_to_listen(asio::ip::udp::v4(), GVISION_SERVER_PORT);
    asio::ip::udp::socket socket(io_service, endpoint_to_listen);
    
    boost::array<char, 1024> receive_buffer;

    /*** Publisher setup ***/
    //ITPS::NonBlockingPublisher<std::string> world_data_pub("GlobalVisionServer", "WorldData");
    ITPS::BlockingPublisher<std::string> world_data_pub("GlobalVisionServer", "WorldData");
    
    
    logger.log(Info, "Server Started on Port Number:" + repr(GVISION_SERVER_PORT) 
                + ", Receiving Global Vision Data");

    size_t num_received;
    try{
        while(true){
            num_received = socket.receive_from(boost::asio::buffer(receive_buffer), endpoint_to_listen);

            std::string packet_received = std::string(receive_buffer.begin(), receive_buffer.begin() + num_received);
            
            // TODO: remove delimiter... I dont know if we still need this since changed to UDP
            // data.erase(--data.end()); 
            // ANS: not need to worry about delimiter because UDP doesn't need delimiter to identify packet length
            
            vis_data.ParseFromString(packet_received);

            logger.log(Info, vis_data.DebugString());
            // world_data_pub.publish(data); // publish serialized data

        }
    }
    catch(std::exception& e)
    {
        logger.log(Error, "[Exception]" + std::string(e.what()));
    }

}