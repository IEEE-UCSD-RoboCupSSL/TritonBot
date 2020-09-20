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
#include "Config/config.hpp"

void VFirmClient::task(ThreadPool& thread_pool) {
   
    B_Log logger;
    logger.add_tag("VFirmClient Module");

    boost::asio::io_service io_service;
    boost::asio::ip::tcp::endpoint ep(boost::asio::ip::address::from_string(vfirm_ip), vfirm_port);
    boost::asio::ip::tcp::socket socket(io_service);
    try {
        // Establish TCP connection with vfirm.exe

        socket.open(boost::asio::ip::tcp::v4());
        socket.connect(ep);
        logger(Info) << "\033[0;32m connection with vfirm.exe established \033[0m";

    }
    catch(std::exception& e)
    {
        logger.log(Error, "\033[0;31m [Exception] "  + std::string(e.what()) + " \033[0m");
    }

    boost::mutex mu;

    // cmd thread
    thread_pool.execute( [&]() {
        B_Log logger;
        logger.add_tag("VFirmClient Module:cmd thread");
        logger(Info) << "\033[0;32m Thread Started \033[0m";
        
        ITPS::Subscriber<VF_Commands> vf_cmd_sub("vfirm-client", "commands", vf_cmd_mq_size);

        while(!vf_cmd_sub.subscribe());
        std::string write;
        VF_Commands cmd;
        try {    
            while(1) { 
                cmd = vf_cmd_sub.pop_msg();
                
                cmd.SerializeToString(&write);

                mu.lock();
                boost::asio::write(socket, boost::asio::buffer(write));
                mu.unlock();
            }
        }
        catch(std::exception& e)
        {
            logger.log(Error, "\033[0;31m [Exception] "  + std::string(e.what()) + " \033[0m");
        }
    });

    // data thread
    thread_pool.execute( [&]() {
        B_Log logger;
        logger.add_tag("VFirmClient Module:data thread");
        logger(Info) << "\033[0;32m Thread Started \033[0m";

        ITPS::Publisher<VF_Data> vf_data_pub("vfirm-client", "data");

        boost::asio::streambuf read_buffer;
        std::istream input_stream(&read_buffer);
        std::string received;

        VF_Data data;
        
        try {
            while(1) {
                
                boost::asio::read_until(socket, read_buffer, "\n");

                mu.lock();
                received = std::string(std::istreambuf_iterator<char>(input_stream), {});
                mu.unlock();
                
                data.ParseFromString(received);
                vf_data_pub.publish(data);
            }
        }
        catch(std::exception& e)
        {
            logger.log(Error, "\033[0;31m [Exception] "  + std::string(e.what()) + " \033[0m");
        }
    });

    
    while(1);
    
}