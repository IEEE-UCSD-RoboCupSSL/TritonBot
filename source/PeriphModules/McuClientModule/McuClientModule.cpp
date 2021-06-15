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

/*====================== helper functions ====================================*/
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

void sendFirmwareCommand(ip::udp::socket& socket, const FirmwareCommand& cmd, ip::udp::endpoint& ep) {
    std::string cmdProtoBinary;
    cmd.SerializeToString(&cmdProtoBinary);
    socket.send_to(asio::buffer(cmdProtoBinary), ep);
}

void sendFirmwareCommand(ip::udp::socket& socket, const ControlOutput& ctrlOut, 
                                const bool turnOnDribbler, const arma::vec2 kickerPwr, ip::udp::endpoint& ep) {
    FirmwareCommand cmd;
    cmd.set_vx((float)ctrlOut.vx);
    cmd.set_vy((float)ctrlOut.vy);
    cmd.set_w((float)ctrlOut.omega);
    cmd.set_dribbler(turnOnDribbler);
    cmd.set_init(false);
    cmd.set_kx((float)kickerPwr(0));
    cmd.set_kz((float)kickerPwr(1));
    sendFirmwareCommand(socket, cmd, ep);
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
/*==========================================================================*/



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
    boost::array<char, UDP_RBUF_SIZE> receiveBuffer;
    std::string writeBuf;

    // MCU Top program will always be at localhost, the top part of the firmware layer will run in the same device this program is at
    asio::ip::tcp::endpoint ep(asio::ip::address::from_string("127.0.0.1"), config.cliConfig.mcuTopTcpPort); 
    std::shared_ptr<asio::ip::tcp::socket> socket; //(ios);
    boost::system::error_code errCode;

    do {
        socket = std::shared_ptr<asio::ip::tcp::socket>(new asio::ip::tcp::socket(ios));
        socket->open(ip::tcp::v4());
        socket->connect(ep, errCode);
        if(errCode) {
            logger(Error) << "Failed at connecting MCU Top, will retry";
        }
    } while(errCode);

    logger.log(Info, "\033[0;32m Connected to MCU Top \033[0m");

    threadPool.execute([&](){
        io_service ios;
        ip::udp::endpoint ep(ip::udp::v4(), config.cliConfig.mcuTopUdpWritePort);        
        std::shared_ptr<asio::ip::udp::socket> udpSocket(new asio::ip::udp::socket(ios));
        udpSocket->open(ip::udp::v4());

        while(true) {
            auto ctrlOut = controlOutputSub.getMsg();
            auto dribCmd = dribblerCommandSub.getMsg();
            auto kickSp = kickerSetPointSub.getMsg();
            /** send all other commands **/
            sendFirmwareCommand(*udpSocket, ctrlOut, dribCmd, kickSp, ep);
            
            delay(TO_PERIOD(MCU_CLIENT_FREQUENCY));
        }
    });

    threadPool.execute([&](){
        io_service ios;
        ip::udp::endpoint ep(ip::udp::v4(), config.cliConfig.mcuTopUdpReadPort);        
        std::shared_ptr<asio::ip::udp::socket> udpSocket(new asio::ip::udp::socket(ios, ep));

        while(true) {
            size_t numReceived = udpSocket->receive_from(asio::buffer(receiveBuffer), ep);
            std::string packetReceived = std::string(receiveBuffer.begin(), receiveBuffer.begin() + numReceived);

            /** convert and publish read packet **/
            auto data = readFirmwareData(packetReceived);
            mcuSensorDataPub.publish(data);

            delay(TO_PERIOD(MCU_CLIENT_FREQUENCY));
        }
    });


    while(true) {
        periodic_session([&](){
            /** handle init command **/
            if(initSensorsCmdSub.getMsg()) {
                // to init or re-init
                asio::write(*socket, asio::buffer("init\n"));
                initSensorsCmdSub.forceSetMsg(false); // set this boolean pub-sub field back to false
                logger(Info) << "\033[0;32m Request command to initialize sensors has been sent to MCU Top program \033[0m";
            }
        }, TO_PERIOD(MCU_CLIENT_FREQUENCY));
    }

    //socket.shutdown(ip::tcp::socket::shutdown_both);
    //socket.close();
}








//-------------------------------------------------------------------------------------//

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