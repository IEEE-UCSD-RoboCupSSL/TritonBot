#include "MicroCtrlerInterface/vfirm_client.hpp"


#include <string>
#include <thread>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include "Utility/systime.hpp"
#include "PubSubModule/thread_pool.hpp"
#include "ProtoGenerated/vFirmware_API.pb.h"
#include "Utility/boost_logger.hpp"





void VFirmClient::task(ThreadPool& thread_pool) {
    B_Log logger;
    /*
    ip = std::string(argv[1]);
    port = std::stoi(std::string(argv[2]), nullptr, 10);

    std::cout << "Successsfully connected to " << ep.address() << " port " << ep.port() << std::endl;
    */
    
    boost::mutex mu;

    // cmd thread
    thread_pool.execute( [&]() {
        /*
        boost::asio::io_service io_service;
        boost::asio::ip::tcp::endpoint ep(boost::asio::ip::address::from_string(ip), port);
        boost::asio::ip::tcp::socket socket(io_service);
        socket.open(boost::asio::ip::tcp::v4());
        socket.connect(ep);
        */
        logger.log(Info, "this is cmd thread\n");
        while(1);
        
    });

    // data thread
    thread_pool.execute( [&]() {
        logger.log(Info, "this is data thread\n");
        while(1);
    });

    

    
}