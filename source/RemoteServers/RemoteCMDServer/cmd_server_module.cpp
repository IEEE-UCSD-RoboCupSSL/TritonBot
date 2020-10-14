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



    io_service.run();
    
}