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


static void handle_sensor_init( boost::mutex *mu, 
                                boost::asio::ip::tcp::socket *socket,
                                bool turn_init_on
                                );


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
        
        ITPS::Subscriber<VF_Commands> vf_cmd_sub("vfirm-client", "commands", vf_cmd_mq_size); //construct with a message queue as buffer
        ITPS::Subscriber<bool> init_sensors_sub("vfirm-client", "init sensors");
        
        while(!vf_cmd_sub.subscribe());
        while(!init_sensors_sub.subscribe());
        init_sensors_sub.add_on_published_callback(boost::bind(&handle_sensor_init, &mu, &socket, _1));
        
        
        std::string write;
        VF_Commands cmd;
        try {    
            while(1) { 
                cmd = vf_cmd_sub.pop_msg(); // To-Do: add timeout
                cmd.set_init(false);
                cmd.SerializeToString(&write);
                write += '\n'; // Don't forget the newline, the server side use it as delim !!!!!
                
                // mu.lock();
                boost::asio::write(socket, boost::asio::buffer(write));
                // mu.unlock();
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
                // mu.lock();
                boost::asio::read_until(socket, read_buffer, "\n");
                // mu.unlock();
                
                received = std::string(std::istreambuf_iterator<char>(input_stream), {});

                
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

static void handle_sensor_init( boost::mutex *mu, 
                                boost::asio::ip::tcp::socket *socket,
                                bool turn_init_on) {
    VF_Commands cmd;
    Vec_2D zero_vec;
    std::string write;
    B_Log logger;


    logger.add_tag("VFirmClient Module:handle sensor init");

    mu->lock();
    
    zero_vec.set_x(0.00);
    zero_vec.set_y(0.00);

    cmd.set_init(turn_init_on);
    cmd.set_allocated_translational_output(&zero_vec);
    cmd.set_rotational_output(0.00);
    cmd.set_allocated_kicker(&zero_vec);
    cmd.set_dribbler(false);

    cmd.SerializeToString(&write);
    write += '\n'; // Don't forget the newline, the server side use it as delim !!!!!
    boost::asio::write(*socket, boost::asio::buffer(write));
    cmd.release_translational_output();  // for every set_allocated_xxx, release is needed to free the memory properly. Happy C++ coding :(  see, java is so awesome :)
    cmd.release_kicker();

    mu->unlock();
    
    logger(Info) << "\033[0;32m Init Sensors \033[0m";
}