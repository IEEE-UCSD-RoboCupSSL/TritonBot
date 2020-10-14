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


// Implementation of task to be run on this thread
void CMDServer::task(ThreadPool& thread_pool) {
    UNUSED(thread_pool); // no child thread is needed in a async scheme

    B_Log logger;
    logger.add_tag("Command Server Module");
    logger(Info) << "\033[0;32m Thread Started \033[0m";

    asio::io_service io_service;

    


    io_service.run();
    
}