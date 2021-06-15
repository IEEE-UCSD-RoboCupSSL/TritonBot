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


FirmwareCommand defaultFirmwareCommand();
void sendFirmwareCommand(ip::tcp::socket& socket, const FirmwareCommand& cmd);
void sendFirmwareCommand(ip::tcp::socket& socket, const ControlOutput& ctrlOut, const bool turnOnDribbler, const arma::vec2 kickerPwr);
McuSensorData readFirmwareData(const std::string& packetStr);
void initSensors(asio::ip::tcp::socket& socket, BLogger& logger);


void McuClientModule::task(ThreadPool& threadPool) {

    BLogger logger;
    logger.addTag("McuClientModule");
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

    asio::streambuf readBuf;
    std::istream inputStream(&readBuf);
    std::string writeBuf;

    // MCU Top program will always be at localhost, the top part of the firmware layer will run in the same device this program is at
    asio::ip::tcp::endpoint ep(asio::ip::address::from_string("127.0.0.1"), config.cliConfig.mcuTopTcpPort); 
    std::shared_ptr<asio::ip::tcp::socket> socket; //(ios);
    boost::system::error_code errCode;

    //delay(2000); // delay needed for connecting to Java part in virtual mode

    do {
        socket = std::shared_ptr<asio::ip::tcp::socket>(new asio::ip::tcp::socket(ios));
        socket->open(ip::tcp::v4());
        socket->connect(ep, errCode);
        if(errCode) {
            logger(Error) << "Failed at connecting MCU Top, will retry";
        }
    } while(errCode);

    logger.log(Info, "\033[0;32m Connected to MCU Top \033[0m");


    while(true) {
        //periodic_session([&](){
            auto ctrlOut = controlOutputSub.getMsg();
            auto dribCmd = dribblerCommandSub.getMsg();
            auto kickSp = kickerSetPointSub.getMsg();


            /** handle init command **/
            if(initSensorsCmdSub.getMsg()) {
                // to init or re-init
                initSensors(*socket, logger);
                initSensorsCmdSub.forceSetMsg(false); // set this boolean pub-sub field back to false
            }
            /** send all other commands **/
            sendFirmwareCommand(*socket, ctrlOut, dribCmd, kickSp);





            /** read a packet **/
            // read_buffer is binded to input_stream
            asio::read_until(*socket, readBuf, "\n"); // read until getting delimiter "\n"
            // convert input stream to string, note that "readBuf" is binded to "input_stream" 
            std::string received =  std::string(std::istreambuf_iterator<char>(inputStream), {}); // c++11 or above
            //std::cout << received << std::endl;

            /** convert and publish read packet **/
            auto data = readFirmwareData(received);
            mcuSensorDataPub.publish(data);
        //}, TO_PERIOD(MCU_CLIENT_FREQUENCY));
        delay(2); // socket read/write freq is not derterministic, so just use a simple delay instead
    }

    //socket.shutdown(ip::tcp::socket::shutdown_both);
    //socket.close();
}


FirmwareCommand defaultFirmwareCommand() {
    FirmwareCommand cmd;
    cmd.set_init(false);
    cmd.set_dribbler(false);
    cmd.set_kx(0.00f);
    cmd.set_kz(0.00f);
    cmd.set_vx(0.00f);
    cmd.set_vy(0.00f);
    cmd.set_w(0.00f);
    return cmd;
}

void sendFirmwareCommand(ip::tcp::socket& socket, const FirmwareCommand& cmd) {
    // std::cout << cmd.DebugString() << std::endl;
    char prefixLength[4];   
    std::string cmdProtoBinary;
    cmd.SerializeToString(&cmdProtoBinary);
    *((int*)prefixLength) = (int)(cmdProtoBinary.length());
    std::string prefix(prefixLength);
    std::string sendStr = prefix + cmdProtoBinary;
    asio::write(socket, asio::buffer(sendStr, sendStr.length()));
    //std::cout << "XXXXX: " << sendStr.length() << std::endl;
    //std::cout << "OOOOO: " << sendStr << std::endl;
    
}

void sendFirmwareCommand(ip::tcp::socket& socket, const ControlOutput& ctrlOut, 
                                const bool turnOnDribbler, const arma::vec2 kickerPwr) {
    FirmwareCommand cmd;
    cmd.set_vx((float)ctrlOut.vx);
    cmd.set_vy((float)ctrlOut.vy);
    cmd.set_w((float)ctrlOut.omega);
    cmd.set_dribbler(turnOnDribbler);
    cmd.set_init(false);
    cmd.set_kx((float)kickerPwr(0));
    cmd.set_kz((float)kickerPwr(1));
    sendFirmwareCommand(socket, cmd);
}

McuSensorData readFirmwareData(const std::string& packetStr) {
    FirmwareData dataReceived;
    McuSensorData data;
    dataReceived.ParseFromString(packetStr);
    data.encCnt = {dataReceived.enc_x(), dataReceived.enc_y()};
    data.imuAcc = {dataReceived.imu_ax(), dataReceived.imu_ay()};
    data.imuOmega = dataReceived.imu_omega();
    data.imuTheta = dataReceived.imu_theta();
    data.isHoldingBall = dataReceived.is_holdingball();
    return data;
}


/* sequence to send a cmd packet through socket to invoke sensor initialization */
void initSensors(asio::ip::tcp::socket& socket, BLogger& logger) {
    FirmwareCommand cmd = defaultFirmwareCommand();
    cmd.set_init(true);
    sendFirmwareCommand(socket, cmd);
    delay(std::chrono::milliseconds(500));
    cmd.set_init(false);
    sendFirmwareCommand(socket, cmd);
    logger(Info) << "\033[0;32m Request command to initialize sensors has been sent to MCU Top program \033[0m";
}




void McuClientModuleMonitor::task(ThreadPool& threadPool) {

    ITPS::FieldSubscriber<McuSensorData> mcuSensorDataSub("From:McuClientModule", "McuSensorData(BodyFrame)");

    ITPS::FieldSubscriber<ControlOutput> controlOutputSub("From:MotionControllerModule", "MotionControlOutput");
    ITPS::FieldSubscriber<bool> dribblerCommandSub("From:CommandProcessorModule", "dribblerSwitch");
    ITPS::FieldSubscriber<arma::vec2> kickerSetPointSub("From:CommandProcessorModule", "KickerSetPoint(On/Off)");
    ITPS::FieldSubscriber<bool> initSensorsCmdSub("From:TcpReceiveModule", "re/init sensors");
   

    try {
        mcuSensorDataSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        controlOutputSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        dribblerCommandSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        kickerSetPointSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        initSensorsCmdSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
    } catch(std::exception& e) {
        BLogger logger;
        logger.addTag("[McuClientModule.cpp]");
        logger.log(Error, std::string(e.what()));
        std::exit(0);
    }

    BLogger mlog;
    mlog.setToShortestFormat();
    while(true) {
        periodic_session([&]() {
            mlog.log(Info, controlOutputSub.getMsg().toString());
            std::stringstream ss;
            ss << "DribblerCommand: " << (dribblerCommandSub.getMsg()?"ON":"OFF") << "\t"
                << "KickerSetPoint: " << kickerSetPointSub.getMsg() << "\t"
                << "InitSensorCommand: " << (initSensorsCmdSub.getMsg()?"True":"False") << std::endl; 
            mlog.log(Info, ss.str());
            mlog.log(Info, mcuSensorDataSub.getMsg().toString());
            mlog(Info) << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<";
        }, std::chrono::milliseconds(samplingPeriod));
    }

}