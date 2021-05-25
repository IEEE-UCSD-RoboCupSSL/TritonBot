#include "PeriphModules/McuClient/DEPRECATED_VFirmClient.hpp"
#include <string>
#include <thread>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include "Misc/PubSubSystem/ThreadPool.hpp"
#include "Misc/Utility/ClockUtil.hpp"
#include "Misc/Utility/BoostLogger.hpp"
#include "Misc/Utility/Common.hpp"
#include "Config/Config.hpp"
#include "Config/ModuleFrequencies.hpp"
#include "ProtoGenerated/vFirmware_API.pb.h"

using namespace boost;


void init_sensors(asio::ip::tcp::socket& socket, BLogger& logger);

// callback functions
static void on_socket_connected(asio::ip::tcp::socket& socket, 
                                std::string& write_buf,
                                asio::streambuf& read_buf, 
                                //ITPS::FieldPublisher<VF_Data>& firm_data_pub, 
                                ITPS::FieldSubscriber<ControlOutput>& controlOutputSub,
                                ITPS::FieldSubscriber<bool>& initSensorsCmdSub,
                                ITPS::FieldSubscriber<bool>& dribblerCommandSub,
                                ITPS::FieldSubscriber<arma::vec2>& kickerSetPointSub,
                                BLogger& logger,  
                                const system::error_code& error);

static void on_data_received(asio::ip::tcp::socket& socket, 
                             std::string& write_buf,
                             asio::streambuf& read_buf, 
                             //ITPS::FieldPublisher<VF_Data>& firm_data_pub, 
                             ITPS::FieldSubscriber<ControlOutput>& controlOutputSub,
                             ITPS::FieldSubscriber<bool>& initSensorsCmdSub,
                             ITPS::FieldSubscriber<bool>& dribblerCommandSub,
                             ITPS::FieldSubscriber<arma::vec2>& kickerSetPointSub,
                             BLogger& logger,  
                             const system::error_code& error);

static void on_cmd_sent(asio::ip::tcp::socket& socket, 
                        std::string& write_buf,
                        asio::streambuf& read_buf, 
                        //ITPS::FieldPublisher<VF_Data>& firm_data_pub, 
                        ITPS::FieldSubscriber<ControlOutput>& controlOutputSub,
                        ITPS::FieldSubscriber<bool>& initSensorsCmdSub,
                        ITPS::FieldSubscriber<bool>& dribblerCommandSub,
                        ITPS::FieldSubscriber<arma::vec2>& kickerSetPointSub,
                        BLogger& logger,  
                        const system::error_code& error);


void VFirmClientModule::task(ThreadPool& threadPool) {

    BLogger logger;
    logger.addTag("VFirmClientModule");
    logger(Info) << "\033[0;32m Thread Started \033[0m";


    ITPS::FieldPublisher<McuSensorData> mcuSensorDataPub("From:McuClientModule", "McuSensorData(BodyFrame)", defaultMcuSensorData());
  

    ITPS::FieldSubscriber<ControlOutput> controlOutputSub("From:MotionControllerModule", "MotionControlOutput");
    ITPS::FieldSubscriber<bool> dribblerCommandSub("From:CommandProcessorModule", "dribblerSwitch");
    ITPS::FieldSubscriber<arma::vec2> kickerSetPointSub("From:CommandProcessorModule", "KickerSetPoint(On/Off)");
    ITPS::FieldSubscriber<bool> initSensorsCmdSub("From:TcpReceiveModule", "re/init sensors");
   
    try {
        controlOutputSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        dribblerCommandSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        kickerSetPointSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        initSensorsCmdSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
    }
    catch(std::exception& e) {
        BLogger logger;
        logger.addTag("[VFirmClientModule.cpp]");
        logger.log(Error, std::string(e.what()));
        std::exit(0);
    }

    asio::io_service io_service;
    asio::ip::tcp::endpoint ep(asio::ip::address::from_string("127.0.0.1"), 8000); //hard code these IP because this legacy file is used for testing purpose only
    asio::ip::tcp::socket socket(io_service);
    asio::streambuf read_buf;
    std::string write_buf;

    logger(Info) << "\033[0;32m Initialized \033[0m";



    try {
        // Establish TCP connection with vfirm.exe
        socket.open(asio::ip::tcp::v4());
        socket.async_connect(ep, boost::bind(&on_socket_connected, 
                                                boost::ref(socket),
                                                boost::ref(write_buf), 
                                                boost::ref(read_buf), 
                                                //boost::ref(firm_data_pub), 
                                                boost::ref(controlOutputSub), 
                                                boost::ref(initSensorsCmdSub),
                                                boost::ref(dribblerCommandSub),
                                                boost::ref(kickerSetPointSub),
                                                boost::ref(logger), 
                                                asio::placeholders::error));
    }
    catch(std::exception& e)
    {
        BLogger logger;
        logger.addTag("[VFirmClientModule.cpp]");
        logger.log(Error, std::string(e.what()));
        std::exit(0);
    }

    io_service.run(); // this line blocks until the async tasks queue becomes empty


}



/** old code styling, too lazy to refactor them since this file is for testing use only  **/


//====================================================================================//
/* callback handler when socket connection is established */
static void on_socket_connected(asio::ip::tcp::socket& socket, 
                                std::string& write_buf,
                                asio::streambuf& read_buf, 
                                //ITPS::FieldPublisher<VF_Data>& firm_data_pub, 
                                ITPS::FieldSubscriber<ControlOutput>& controlOutputSub,
                                ITPS::FieldSubscriber<bool>& initSensorsCmdSub,
                                ITPS::FieldSubscriber<bool>& dribblerCommandSub,
                                ITPS::FieldSubscriber<arma::vec2>& kickerSetPointSub,
                                BLogger& logger,  
                                const system::error_code& error) {
    if(error) {
        BLogger logger;
        logger.addTag("[vfirm_client.cpp]");
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
                                            //boost::ref(firm_data_pub), 
                                            boost::ref(controlOutputSub), 
                                            boost::ref(initSensorsCmdSub),
                                            boost::ref(dribblerCommandSub),
                                            boost::ref(kickerSetPointSub),
                                            boost::ref(logger), 
                                            asio::placeholders::error)); 
}


/* callback handler when socket received a new data packet */
static void on_data_received(asio::ip::tcp::socket& socket, 
                             std::string& write_buf,
                             asio::streambuf& read_buf, 
                             //ITPS::FieldPublisher<VF_Data>& firm_data_pub, 
                             ITPS::FieldSubscriber<ControlOutput>& controlOutputSub,
                             ITPS::FieldSubscriber<bool>& initSensorsCmdSub,
                             ITPS::FieldSubscriber<bool>& dribblerCommandSub,
                             ITPS::FieldSubscriber<arma::vec2>& kickerSetPointSub,
                             BLogger& logger,  
                             const system::error_code& error) {
    if(error) {
        BLogger logger;
        logger.addTag("[vfirm_client.cpp]");
        logger.log(Error, error.message());
        std::exit(0);
    }

    VF_Data data;                             
    std::istream input_stream(&read_buf); 
    std::string received;

    // where is read_buf used? checkout few lines above
    received = std::string(std::istreambuf_iterator<char>(input_stream), {});            
    data.ParseFromString(received);

    ///firm_data_pub.publish(data); // EKF module is subscribing to this module.

    logger.log( Debug, "Trans_Dis: " + repr(data.translational_displacement().x()) + ' ' + repr(data.translational_displacement().y()));
    logger.log( Debug, "Trans_Vel:" + repr(data.translational_velocity().x()) + ' ' + repr(data.translational_velocity().y()));
    logger.log( Debug, "Rot_Dis:" + repr(data.rotational_displacement()));
    logger.log( Debug, "Rot_Vel:" + repr(data.rotational_velocity()) + "\n :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) ");
    


    bool re_init = initSensorsCmdSub.getMsg(); // non blocking
    if(re_init) {
        init_sensors(socket, logger);
        initSensorsCmdSub.forceSetMsg(false);
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
    ControlOutput co = controlOutputSub.getMsg();
    auto ko = kickerSetPointSub.getMsg();
    Vec_2D tout, kout;
    tout.set_x(co.vx);
    tout.set_y(co.vy);
    kout.set_x(ko(0));
    kout.set_y(ko(1));
    cmd.set_allocated_translational_output(&tout);
    cmd.set_rotational_output(co.omega);
    cmd.set_allocated_kicker(&kout);
    cmd.set_dribbler(dribblerCommandSub.getMsg());
    //cmd = controlOutputSub.pop_msg(FIRM_CMD_SUB_TIMEOUT, default_cmd);

    write_buf.clear(); // clear the string (as a std buffer)
    cmd.set_init(false);
    cmd.SerializeToString(&write_buf);
    write_buf += "\n"; // Don't forget the newline, the server side use it as delim !!!!!

    default_cmd.release_translational_output();  // memory headache, Happy C++ coding :(  see, java is so awesome :)
    default_cmd.release_kicker();
    cmd.release_translational_output();
    cmd.release_kicker();

    // set the write event
    boost::asio::async_write(socket, asio::buffer(write_buf), 
                             boost::bind(&on_cmd_sent, 
                                        boost::ref(socket),
                                        boost::ref(write_buf), 
                                        boost::ref(read_buf), 
                                        //boost::ref(firm_data_pub), 
                                        boost::ref(controlOutputSub),
                                        boost::ref(initSensorsCmdSub),
                                        boost::ref(dribblerCommandSub),
                                        boost::ref(kickerSetPointSub),
                                        boost::ref(logger), 
                                        asio::placeholders::error));

}



/* callback handler when socket finished sending a command */
static void on_cmd_sent(asio::ip::tcp::socket& socket, 
                        std::string& write_buf,
                        asio::streambuf& read_buf, 
                        //ITPS::FieldPublisher<VF_Data>& firm_data_pub, 
                        ITPS::FieldSubscriber<ControlOutput>& controlOutputSub,
                        ITPS::FieldSubscriber<bool>& initSensorsCmdSub,
                        ITPS::FieldSubscriber<bool>& dribblerCommandSub,
                        ITPS::FieldSubscriber<arma::vec2>& kickerSetPointSub,
                        BLogger& logger,  
                        const system::error_code& error) {
    if(error) {
        BLogger logger;
        logger.addTag("[vfirm_client.cpp]");
        logger.log(Error, error.message());
        std::exit(0);
    } 

    // set the next read event
    asio::async_read_until(socket, read_buf, "\n",
                            boost::bind(&on_data_received, 
                                        boost::ref(socket),
                                        boost::ref(write_buf), 
                                        boost::ref(read_buf), 
                                        //boost::ref(firm_data_pub), 
                                        boost::ref(controlOutputSub), 
                                        boost::ref(initSensorsCmdSub),
                                        boost::ref(dribblerCommandSub),
                                        boost::ref(kickerSetPointSub),
                                        boost::ref(logger), 
                                        asio::placeholders::error)); 
}
//====================================================================================//

/* sequence to send a cmd packet through socket to invoke sensor initialization */
void init_sensors(asio::ip::tcp::socket& socket, BLogger& logger) {
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