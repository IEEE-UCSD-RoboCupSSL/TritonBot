/**
 *  Inter-Chip-Interface:
 *  Module overview:
 *      1. Establish an async TCP connection with the Embedded MCU layer.
 *      2. On successful connection, read data from the server(which is from the Embedded MCU layer)
 *      3. On successful data read, publish data to EKF module and listen to Control Algorithm module
 * to get commands to be sent to the server.
 *      4. On successful command send, make recursive call to "async_read_until" with "on_data_received" 
 * as callback
 **/

#include "PeriphModules/FirmClientModule/FirmClientModule.hpp"

#include <string>
#include <thread>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>

#include "Misc/Utility/Systime.hpp"
#include "Misc/Utility/BoostLogger.hpp"
#include "Misc/Utility/Common.hpp"
#include "Misc/PubSubSystem/ThreadPool.hpp"
#include "Config/Config.hpp"
#include "ProtoGenerated/vFirmware_API.pb.h"

using namespace boost;


//========================== Local Function Declaration ==============================//

static void init_sensors(asio::ip::tcp::socket& socket, B_Log& logger);

// callback functions
static void on_socket_connected(asio::ip::tcp::socket& socket, 
                                std::string& write_buf,
                                asio::streambuf& read_buf, 
                                ITPS::BlockingPublisher<VF_Data>& firm_data_pub, 
                                ITPS::BlockingSubscriber<VF_Commands>& firm_cmd_sub,
                                ITPS::NonBlockingSubscriber<bool>& init_sensors_sub,
                                B_Log& logger,  
                                const system::error_code& error);

static void on_data_received(asio::ip::tcp::socket& socket, 
                             std::string& write_buf,
                             asio::streambuf& read_buf, 
                             ITPS::BlockingPublisher<VF_Data>& firm_data_pub, 
                             ITPS::BlockingSubscriber<VF_Commands>& firm_cmd_sub,
                             ITPS::NonBlockingSubscriber<bool>& init_sensors_sub,
                             B_Log& logger,  
                             const system::error_code& error);

static void on_cmd_sent(asio::ip::tcp::socket& socket, 
                        std::string& write_buf,
                        asio::streambuf& read_buf, 
                        ITPS::BlockingPublisher<VF_Data>& firm_data_pub, 
                        ITPS::BlockingSubscriber<VF_Commands>& firm_cmd_sub,
                        ITPS::NonBlockingSubscriber<bool>& init_sensors_sub,
                        B_Log& logger,  
                        const system::error_code& error);
//====================================================================================//




//====================================================================================//
/* Main Task to be run on a new thread from the thread pool */
void VFirmClient::task(ThreadPool& threadPool) {
    UNUSED(threadPool); // no child thread is needed in a async scheme

    B_Log logger;
    logger.add_tag("VFirmClient Module");
    logger(Info) << "\033[0;32m Thread Started \033[0m";

    asio::io_service io_service;
    asio::ip::tcp::endpoint ep(asio::ip::address::from_string(VFIRM_IP_ADDR), VFIRM_IP_PORT);
    asio::ip::tcp::socket socket(io_service);
    asio::streambuf read_buf;
    std::string write_buf;

    // publisher to publish data sent from vfirm: [vfirm socket] => [firm_data_pub] => [EKF module]
    ITPS::BlockingPublisher<VF_Data> firm_data_pub("FirmClient", "InternalSensorData");

    // subscriber to listen to commands to be sent to vfirm:  [control module] => [firm_cmd_sub] => vfirm socket
    ITPS::BlockingSubscriber<VF_Commands> firm_cmd_sub("FirmClient", "Commands", FIRM_CMD_MQ_SIZE); //construct with a message queue as buffer

    // subscriber to listen to a signal to trigger sensor re/initilization sequence 
    ITPS::NonBlockingSubscriber<bool> init_sensors_sub("vfirm-client", "re/init sensors");

    try {
        firm_cmd_sub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        init_sensors_sub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
    }
    catch(std::exception& e) {
        B_Log logger;
        logger.add_tag("[vfirm_client.cpp]");
        logger.log(Error, e.what());
        std::exit(0);
    }

    logger(Info) << "\033[0;32m Initialized \033[0m";

    try {
        // Establish TCP connection with vfirm.exe
        socket.open(asio::ip::tcp::v4());
        socket.async_connect(ep, boost::bind(&on_socket_connected, 
                                                boost::ref(socket),
                                                boost::ref(write_buf), 
                                                boost::ref(read_buf), 
                                                boost::ref(firm_data_pub), 
                                                boost::ref(firm_cmd_sub), 
                                                boost::ref(init_sensors_sub),
                                                boost::ref(logger), 
                                                asio::placeholders::error));
    }
    catch(std::exception& e)
    {
        B_Log logger;
        logger.add_tag("[vfirm_client.cpp]");
        logger.log(Error, e.what());
        std::exit(0);
    }

    io_service.run(); // this line blocks until the async tasks queue becomes empty
}
//====================================================================================//




//====================================================================================//
/* callback handler when socket connection is established */
static void on_socket_connected(asio::ip::tcp::socket& socket, 
                                std::string& write_buf,
                                asio::streambuf& read_buf, 
                                ITPS::BlockingPublisher<VF_Data>& firm_data_pub, 
                                ITPS::BlockingSubscriber<VF_Commands>& firm_cmd_sub,
                                ITPS::NonBlockingSubscriber<bool>& init_sensors_sub,
                                B_Log& logger,  
                                const system::error_code& error) {
    if(error) {
        B_Log logger;
        logger.add_tag("[vfirm_client.cpp]");
        logger.log(Error, error.message());
        std::exit(0);
    }
    logger(Info) << "\033[0;32m socket connected \033[0m";


    /* start the infinite cycle of [read 1 data] => [send 1 cmd] => [read 1 data] => [send 1 cmd] ....
        * by first begin with a read from the socket
        */
    asio::async_read_until(socket, read_buf, "\n",
                                boost::bind(&on_data_received, 
                                            boost::ref(socket),
                                            boost::ref(write_buf), 
                                            boost::ref(read_buf), 
                                            boost::ref(firm_data_pub), 
                                            boost::ref(firm_cmd_sub), 
                                            boost::ref(init_sensors_sub),
                                            boost::ref(logger), 
                                            asio::placeholders::error)); 
}


/* callback handler when socket received a new data packet */
static void on_data_received(asio::ip::tcp::socket& socket, 
                             std::string& write_buf,
                             asio::streambuf& read_buf, 
                             ITPS::BlockingPublisher<VF_Data>& firm_data_pub, 
                             ITPS::BlockingSubscriber<VF_Commands>& firm_cmd_sub,
                             ITPS::NonBlockingSubscriber<bool>& init_sensors_sub,
                             B_Log& logger,  
                             const system::error_code& error) {
    if(error) {
        B_Log logger;
        logger.add_tag("[vfirm_client.cpp]");
        logger.log(Error, error.message());
        std::exit(0);
    }
    VF_Data data;                             
    std::istream input_stream(&read_buf); // check me
    std::string received;

    // where is read_buf used? checkout few lines above
    received = std::string(std::istreambuf_iterator<char>(input_stream), {});            
    data.ParseFromString(received);

    firm_data_pub.publish(data); // EKF module is subscribing to this module.

    logger.log( Debug, "Trans_Dis: " + repr(data.translational_displacement().x()) + ' ' + repr(data.translational_displacement().y()));
    logger.log( Debug, "Trans_Vel:" + repr(data.translational_velocity().x()) + ' ' + repr(data.translational_velocity().y()));
    logger.log( Debug, "Rot_Dis:" + repr(data.rotational_displacement()));
    logger.log( Debug, "Rot_Vel:" + repr(data.rotational_velocity()) + "\n :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) ");
    

    bool re_init = init_sensors_sub.latest_msg(); // non blocking
    if(re_init) {
        init_sensors(socket, logger);
        init_sensors_sub.force_set_latest_msg(false);
    }


    Vec_2D zero_vec;
    VF_Commands default_cmd;

    zero_vec.set_x(0.00);
    zero_vec.set_y(0.00);

    default_cmd.set_init(false);
    default_cmd.set_allocated_translational_output(&zero_vec);
    default_cmd.set_rotational_output(0.00);
    default_cmd.set_allocated_kicker(&zero_vec);
    default_cmd.set_dribbler(false);    

    VF_Commands cmd;
    // conditionally blocking (this method blocks when the message queue is empty)
    cmd = firm_cmd_sub.pop_msg(FIRM_CMD_SUB_TIMEOUT, default_cmd);

    write_buf.clear(); // clear the string (as a std buffer)
    cmd.set_init(false);
    cmd.SerializeToString(&write_buf);
    write_buf += "\n"; // Don't forget the newline, the server side use it as delim !!!!!

    default_cmd.release_translational_output();  // memory headache, Happy C++ coding :(  see, java is so awesome :)
    default_cmd.release_kicker();


    // set the write event
    boost::asio::async_write(socket, asio::buffer(write_buf), 
                             boost::bind(&on_cmd_sent, 
                                        boost::ref(socket),
                                        boost::ref(write_buf), 
                                        boost::ref(read_buf), 
                                        boost::ref(firm_data_pub), 
                                        boost::ref(firm_cmd_sub),
                                        boost::ref(init_sensors_sub),
                                        boost::ref(logger), 
                                        asio::placeholders::error));

}



/* callback handler when socket finished sending a command */
static void on_cmd_sent(asio::ip::tcp::socket& socket, 
                        std::string& write_buf,
                        asio::streambuf& read_buf, 
                        ITPS::BlockingPublisher<VF_Data>& firm_data_pub, 
                        ITPS::BlockingSubscriber<VF_Commands>& firm_cmd_sub,
                        ITPS::NonBlockingSubscriber<bool>& init_sensors_sub,
                        B_Log& logger,  
                        const system::error_code& error) {
    if(error) {
        B_Log logger;
        logger.add_tag("[vfirm_client.cpp]");
        logger.log(Error, error.message());
        std::exit(0);
    } 

    // set the next read event
    asio::async_read_until(socket, read_buf, "\n",
                            boost::bind(&on_data_received, 
                                        boost::ref(socket),
                                        boost::ref(write_buf), 
                                        boost::ref(read_buf), 
                                        boost::ref(firm_data_pub), 
                                        boost::ref(firm_cmd_sub), 
                                        boost::ref(init_sensors_sub),
                                        boost::ref(logger), 
                                        asio::placeholders::error)); 
}
//====================================================================================//

/* sequence to send a cmd packet through socket to invoke sensor initialization */
static void init_sensors(asio::ip::tcp::socket& socket, B_Log& logger) {
    VF_Commands cmd;
    Vec_2D zero_vec;
    std::string write;
    
    zero_vec.set_x(0.00);
    zero_vec.set_y(0.00);

    cmd.set_init(true);
    cmd.set_allocated_translational_output(&zero_vec);
    cmd.set_rotational_output(0.00);
    cmd.set_allocated_kicker(&zero_vec);
    cmd.set_dribbler(false);
    cmd.SerializeToString(&write);
    write += '\n'; // Don't forget the newline, the server side use it as delim !!!!!
    boost::asio::write(socket, boost::asio::buffer(write));
    delay(500);

    cmd.set_init(false);
    cmd.SerializeToString(&write);
    write += '\n'; // Don't forget the newline, the server side use it as delim !!!!!
    boost::asio::write(socket, boost::asio::buffer(write));

    cmd.release_translational_output();  // memory headache, Happy C++ coding :(  see, java is so awesome :)
    cmd.release_kicker();
    logger(Info) << "\033[0;32m request to initialize sensors is sent to vfirm \033[0m";
}