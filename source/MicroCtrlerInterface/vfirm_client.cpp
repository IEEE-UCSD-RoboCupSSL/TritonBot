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
#include "Utility/common.hpp"

using namespace boost;

static void on_socket_connected(B_Log& logger, const system::error_code& error) {
    logger(Info) << "\033[0;32m socket connected \033[0m";
}

// static void init_sensor_packet(VF_Commands& cmd) {
//     Vec_2D zero_vec;
//     std::string write;
    
//     zero_vec.set_x(0.00);
//     zero_vec.set_y(0.00);

//     cmd.set_init(true);
//     cmd.set_allocated_translational_output(&zero_vec);
//     cmd.set_rotational_output(0.00);
//     cmd.set_allocated_kicker(&zero_vec);
//     cmd.set_dribbler(false);

//     cmd.SerializeToString(&write);
//     write += '\n'; // Don't forget the newline, the server side use it as delim !!!!!
//     asio::write(*socket, asio::buffer(write));
//     cmd.release_translational_output();  // for every set_allocated_xxx, release is needed to free the memory properly. Happy C++ coding :(  see, java is so awesome :)
//     cmd.release_kicker();
// }
static void on_data_received(asio::ip::tcp::socket& socket, asio::streambuf& read_buf, 
                             ITPS::Publisher<VF_Data>& vf_data_pub, B_Log& logger,  
                             const system::error_code& error) {
    VF_Data data;                             
    asio::streambuf read_buffer;
    std::istream input_stream(&read_buffer); // check me
    std::string received;

    // where is read_buf used? checkout few lines above
    received = std::string(std::istreambuf_iterator<char>(input_stream), {});            
    data.ParseFromString(received);
    vf_data_pub.publish(data);
    logger.log( Info, "Trans_Dis: " + repr(data.translational_displacement().x()) + ' ' + repr(data.translational_displacement().y()));
    logger.log( Info, "Trans_Vel:" + repr(data.translational_velocity().x()) + ' ' + repr(data.translational_velocity().y()));
    logger.log( Info, "Rot_Dis:" + repr(data.rotational_displacement()));
    logger.log( Info, "Rot_Vel:" + repr(data.rotational_velocity()) + "\n :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) ");
    
    asio::async_read_until(socket, read_buf, "\n",
                        boost::bind(&on_data_received, boost::ref(socket), boost::ref(read_buf), 
                                    boost::ref(vf_data_pub), boost::ref(logger), 
                                    asio::placeholders::error));

}
static void on_cmd_sent();


void VFirmClient::task(ThreadPool& thread_pool) {
   
    B_Log logger;
    logger.add_tag("VFirmClient Module");

    asio::io_service io_service;
    asio::ip::tcp::endpoint ep(asio::ip::address::from_string(vfirm_ip), vfirm_port);
    asio::ip::tcp::socket socket(io_service);
    asio::streambuf read_buf;

    ITPS::Publisher<VF_Data> vf_data_pub("vfirm-client", "data");
    
    try {
        // Establish TCP connection with vfirm.exe

        socket.open(asio::ip::tcp::v4());
        socket.async_connect(ep, boost::bind(&on_socket_connected, boost::ref(logger), asio::placeholders::error));
         
        // socket.async_write_some();

        asio::async_read_until(socket, read_buf, "\n",
                                    boost::bind(&on_data_received, boost::ref(socket), boost::ref(read_buf), 
                                    boost::ref(vf_data_pub), boost::ref(logger), 
                                    asio::placeholders::error)); 
    }
    catch(std::exception& e)
    {
        logger.log(Error, "\033[0;31m [Exception] "  + std::string(e.what()) + " \033[0m");
    }

    io_service.run();


    // mutex mu;

    // // cmd thread
    // thread_pool.execute( [&]() {
    //     B_Log logger;
    //     logger.add_tag("VFirmClient Module:cmd thread");
    //     logger(Info) << "\033[0;32m Thread Started \033[0m";
        
    //     ITPS::Subscriber<VF_Commands> vf_cmd_sub("vfirm-client", "commands", vf_cmd_mq_size); //construct with a message queue as buffer
    //     ITPS::Subscriber<bool> init_sensors_sub("vfirm-client", "init sensors");
        
    //     while(!vf_cmd_sub.subscribe());
    //     while(!init_sensors_sub.subscribe());
    //     init_sensors_sub.add_on_published_callback(bind(&handle_sensor_init, &mu, &socket, _1));
        
        
    //     std::string write;
    //     VF_Commands cmd;
    //     try {    
    //         while(1) { 
    //             cmd = vf_cmd_sub.pop_msg(); // To-Do: add timeout
    //             cmd.set_init(false);
    //             cmd.SerializeToString(&write);
    //             write += '\n'; // Don't forget the newline, the server side use it as delim !!!!!
                
    //             // mu.lock();
    //             asio::write(socket, asio::buffer(write));
    //             // mu.unlock();
    //         }
    //     }
    //     catch(std::exception& e)
    //     {
    //         logger.log(Error, "\033[0;31m [Exception] "  + std::string(e.what()) + " \033[0m");
    //     }
    // });

    // // data thread
    // thread_pool.execute( [&]() {
    //     B_Log logger;
    //     logger.add_tag("VFirmClient Module:data thread");
    //     logger(Info) << "\033[0;32m Thread Started \033[0m";

    //     ITPS::Publisher<VF_Data> vf_data_pub("vfirm-client", "data");

    //     asio::streambuf read_buffer;
    //     std::istream input_stream(&read_buffer);
    //     std::string received;

    //     VF_Data data;
        
    //     try {
    //         while(1) {
    //             // mu.lock();
    //             asio::read_until(socket, read_buffer, "\n");
    //             // mu.unlock();
                
    //             received = std::string(std::istreambuf_iterator<char>(input_stream), {});

                
    //             data.ParseFromString(received);
    //             vf_data_pub.publish(data);

    //         }
    //     }
    //     catch(std::exception& e)
    //     {
    //         logger.log(Error, "\033[0;31m [Exception] "  + std::string(e.what()) + " \033[0m");
    //     }
    // });

    
}
