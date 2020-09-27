#include "MicroCtrlerClient/vfirm_client.hpp"


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

static VF_Commands default_cmd;

//========================== Local Function Declaration ==============================//

static void init_sensors(asio::ip::tcp::socket& socket, B_Log& logger);

// callback functions
static void on_socket_connected(asio::ip::tcp::socket& socket, 
                                std::string& write_buf,
                                asio::streambuf& read_buf, 
                                ITPS::Publisher<VF_Data>& vf_data_pub, 
                                ITPS::Subscriber<VF_Commands>& vf_cmd_sub,
                                ITPS::Subscriber<bool>& init_sensors_sub,
                                B_Log& logger,  
                                const system::error_code& error);

static void on_data_received(asio::ip::tcp::socket& socket, 
                             std::string& write_buf,
                             asio::streambuf& read_buf, 
                             ITPS::Publisher<VF_Data>& vf_data_pub, 
                             ITPS::Subscriber<VF_Commands>& vf_cmd_sub,
                             ITPS::Subscriber<bool>& init_sensors_sub,
                             B_Log& logger,  
                             const system::error_code& error);

static void on_cmd_sent(asio::ip::tcp::socket& socket, 
                        std::string& write_buf,
                        asio::streambuf& read_buf, 
                        ITPS::Publisher<VF_Data>& vf_data_pub, 
                        ITPS::Subscriber<VF_Commands>& vf_cmd_sub,
                        ITPS::Subscriber<bool>& init_sensors_sub,
                        B_Log& logger,  
                        const system::error_code& error);
//====================================================================================//




//====================================================================================//
/* Main Task to be run on a new thread from the thread pool */
void VFirmClient::task(ThreadPool& thread_pool) {
    UNUSED(thread_pool); // no child thread is needed in a async scheme

    B_Log logger;
    logger.add_tag("VFirmClient Module");
    logger(Info) << "\033[0;32m Thread Started \033[0m";

    asio::io_service io_service;
    asio::ip::tcp::endpoint ep(asio::ip::address::from_string(VFIRM_IP_ADDR), VFIRM_IP_PORT);
    asio::ip::tcp::socket socket(io_service);
    asio::streambuf read_buf;
    std::string write_buf;

    // publisher to publish data sent from vfirm: [vfirm socket] => [vf_data_pub] => [EKF module]
    ITPS::Publisher<VF_Data> vf_data_pub("vfirm-client", "data");

    // subscriber to listen to commands to be sent to vfirm:  [control module] => [vf_cmd_sub] => vfirm socket
    ITPS::Subscriber<VF_Commands> vf_cmd_sub("vfirm-client", "commands", VF_CMD_MQ_SIZE); //construct with a message queue as buffer

    // subscriber to listen to a signal to trigger sensor re/initilization sequence 
    ITPS::Subscriber<bool> init_sensors_sub("vfirm-client", "re/init sensors");

    while(!vf_cmd_sub.subscribe());
    while(!init_sensors_sub.subscribe());

    try {
        // Establish TCP connection with vfirm.exe
        socket.open(asio::ip::tcp::v4());
        socket.async_connect(ep, boost::bind(&on_socket_connected, 
                                                boost::ref(socket),
                                                boost::ref(write_buf), 
                                                boost::ref(read_buf), 
                                                boost::ref(vf_data_pub), 
                                                boost::ref(vf_cmd_sub), 
                                                boost::ref(init_sensors_sub),
                                                boost::ref(logger), 
                                                asio::placeholders::error));
    }
    catch(std::exception& e)
    {
        logger.log(Error, "\033[0;31m [Exception] "  + std::string(e.what()) + " \033[0m");
    }

    io_service.run(); // this line blocks until the async tasks queue becomes empty
}
//====================================================================================//




//====================================================================================//
/* callback handler when socket connection is established */
static void on_socket_connected(asio::ip::tcp::socket& socket, 
                                std::string& write_buf,
                                asio::streambuf& read_buf, 
                                ITPS::Publisher<VF_Data>& vf_data_pub, 
                                ITPS::Subscriber<VF_Commands>& vf_cmd_sub,
                                ITPS::Subscriber<bool>& init_sensors_sub,
                                B_Log& logger,  
                                const system::error_code& error) {
    if(error) {
        logger.log(Error, error.message());
        return;
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
                                            boost::ref(vf_data_pub), 
                                            boost::ref(vf_cmd_sub), 
                                            boost::ref(init_sensors_sub),
                                            boost::ref(logger), 
                                            asio::placeholders::error)); 
}



/* callback handler when socket received a new data packet */
static void on_data_received(asio::ip::tcp::socket& socket, 
                             std::string& write_buf,
                             asio::streambuf& read_buf, 
                             ITPS::Publisher<VF_Data>& vf_data_pub, 
                             ITPS::Subscriber<VF_Commands>& vf_cmd_sub,
                             ITPS::Subscriber<bool>& init_sensors_sub,
                             B_Log& logger,  
                             const system::error_code& error) {
    if(error) {
        logger.log(Error, error.message());
        return;
    }
    VF_Data data;                             
    std::istream input_stream(&read_buf); // check me
    std::string received;

    // where is read_buf used? checkout few lines above
    received = std::string(std::istreambuf_iterator<char>(input_stream), {});            
    data.ParseFromString(received);

    vf_data_pub.publish(data);

    logger.log( Debug, "Trans_Dis: " + repr(data.translational_displacement().x()) + ' ' + repr(data.translational_displacement().y()));
    logger.log( Debug, "Trans_Vel:" + repr(data.translational_velocity().x()) + ' ' + repr(data.translational_velocity().y()));
    logger.log( Debug, "Rot_Dis:" + repr(data.rotational_displacement()));
    logger.log( Debug, "Rot_Vel:" + repr(data.rotational_velocity()) + "\n :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) ");
    

    bool re_init = init_sensors_sub.latest_msg(); // non blocking
    if(re_init) {
        init_sensors(socket, logger);
        init_sensors_sub.reset_latest_msg_sink(false);
    }


    VF_Commands cmd;
    // conditionally blocking (this method blocks when the message queue is empty)
    cmd = vf_cmd_sub.pop_msg(VF_CMD_SUB_TIMEOUT, default_cmd);

    write_buf.clear(); // clear the string (as a std buffer)
    cmd.set_init(false);
    cmd.SerializeToString(&write_buf);
    write_buf += "\n"; // Don't forget the newline, the server side use it as delim !!!!!

    // set the write event
    boost::asio::async_write(socket, asio::buffer(write_buf), 
                             boost::bind(&on_cmd_sent, 
                                        boost::ref(socket),
                                        boost::ref(write_buf), 
                                        boost::ref(read_buf), 
                                        boost::ref(vf_data_pub), 
                                        boost::ref(vf_cmd_sub),
                                        boost::ref(init_sensors_sub),
                                        boost::ref(logger), 
                                        asio::placeholders::error));

}



/* callback handler when socket finished sending a command */
static void on_cmd_sent(asio::ip::tcp::socket& socket, 
                        std::string& write_buf,
                        asio::streambuf& read_buf, 
                        ITPS::Publisher<VF_Data>& vf_data_pub, 
                        ITPS::Subscriber<VF_Commands>& vf_cmd_sub,
                        ITPS::Subscriber<bool>& init_sensors_sub,
                        B_Log& logger,  
                        const system::error_code& error) {
    if(error) {
        logger.log(Error, error.message());
        return;
    } 

    // set the next read event
    asio::async_read_until(socket, read_buf, "\n",
                            boost::bind(&on_data_received, 
                                        boost::ref(socket),
                                        boost::ref(write_buf), 
                                        boost::ref(read_buf), 
                                        boost::ref(vf_data_pub), 
                                        boost::ref(vf_cmd_sub), 
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

    default_cmd = cmd; // make a copy

    cmd.release_translational_output();  // for every set_allocated_xxx, release is needed to free the memory properly. Happy C++ coding :(  see, java is so awesome :)
    cmd.release_kicker();
    logger(Info) << "\033[0;32m request to initialize sensors is sent to vfirm \033[0m";
}