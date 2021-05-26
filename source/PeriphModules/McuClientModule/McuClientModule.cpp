#include "PeriphModules/McuClientModule/McuClientModule.hpp"
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
#include "ProtoGenerated/FirmwareAPI.pb.h"

using namespace boost;
using namespace boost::asio;


void McuClientModule::task(ThreadPool& threadPool) {

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
        logger.addTag("[McuClientModule.cpp]");
        logger.log(Error, std::string(e.what()));
        std::exit(0);
    }

    logger(Info) << "\033[0;32m Initialized \033[0m";

    asio::io_service ios;

    // MCU Top program will always be at localhost, the top part of the firmware layer will run in the same device this program is at
    asio::ip::tcp::endpoint ep(asio::ip::address::from_string("127.0.0.1"), config.cliConfig.mcuTopTcpPort); 
    asio::ip::tcp::socket socket(ios);
    asio::streambuf readBuf;
    std::istream inputStream(&readBuf);
    std::string writeBuf;
    socket.open(ip::tcp::v4());
    socket.connect(ep);

    logger.log(Info, "\033[0;32m Connected to MCU Top \033[0m");


    while(true) {
        periodic_session([&](){
            // read_buffer is binded to input_stream
            asio::read_until(socket, readBuf, "\n"); // read until getting delimiter "\n"
            // convert input stream to string, note that "readBuf" is binded to "input_stream" 
            std::string received =  std::string(std::istreambuf_iterator<char>(inputStream), {}); // c++11 or above
            std::cout << received << std::endl;

            asio::write(socket, buffer("Hello World!\n"));
        }, TO_PERIOD(MCU_CLIENT_FREQUENCY));
    }

    //socket.shutdown(ip::tcp::socket::shutdown_both);
    //socket.close();
}

