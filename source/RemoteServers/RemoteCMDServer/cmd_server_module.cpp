#include "RemoteServers/RemoteCMDServer/cmd_server_module.hpp"

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
using namespace boost::asio;
using namespace boost::asio::ip;

// Implementation of task to be run on this thread
void CMDServer::task(ThreadPool& thread_pool) {
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

    logger.log(Info, "CMD Server Started on Port Number:" + repr(CMD_SERVER_PORT) 
                + ", Listening to Remote AI Commands... ");
    Commands cmd;

    while(1) {
        num_received = socket.receive_from(asio::buffer(receive_buffer), ep_listen);
        packet_received = std::string(receive_buffer.begin(), receive_buffer.begin() + num_received);
        logger.log(Info, packet_received);
        cmd.ParseFromString(packet_received);
        // ...
    }

    io_service.run();
    
}