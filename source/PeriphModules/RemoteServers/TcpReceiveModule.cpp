#include "PeriphModules/RemoteServers/TcpReceiveModule.hpp"
#include <string>
#include <thread>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include "Misc/PubSubSystem/ThreadPool.hpp"
#include "Misc/Utility/Systime.hpp"
#include "Misc/Utility/BoostLogger.hpp"
#include "Misc/Utility/Common.hpp"
#include "Config/Config.hpp"
#include "ProtoGenerated/RemoteAPI.pb.h"
#include "Config/ModuleFrequencies.hpp"

using namespace boost;

boost::mutex mu;


static void backgndTask(ITPS::FieldSubscriber<bool>& ballcapStatusSub, 
                         asio::ip::tcp::socket& socket) {
    bool prevBallCapStatus = true; // deliberately set it true to have a extra socket send at the begining
    auto period = TO_PERIOD(TCP_RECEIVE_FREQUENCY);
    while(true) { 
        auto t = CHRONO_NOW;

        std::string sendStr;
        bool ballcapStatus;
        ballcapStatus = ballcapStatusSub.latest_msg(); 
        if(ballcapStatus != prevBallCapStatus) {
            if(ballcapStatus) {
                sendStr = "BallOnHold";
            }
            else {
                sendStr = "BallOffHold";
            }
            mu.lock();
            asio::write(socket, asio::buffer(sendStr + "\n"));
            mu.unlock();
        }
        prevBallCapStatus = ballcapStatus;

        std::this_thread::sleep_until(t + period);   
    }
}


// Implementation of task to be run on this thread
void TcpReceiveModule::task(ThreadPool& threadPool) {
    BLogger logger;
    logger.addTag("Connection Server Module");
    logger(Info) << "\033[0;32m Thread Started \033[0m";


    int fieldLength = 0;
    int fieldWidth = 0;
    int goalDepth = 0;
    int goalWidth = 0;

    asio::io_service ios;
    asio::ip::tcp::endpoint endpoint(asio::ip::tcp::v4(), TCP_PORT);
    asio::ip::tcp::acceptor acceptor(ios, endpoint);
    asio::ip::tcp::socket socket(ios);
    asio::streambuf read_buf;
    std::string write_buf;



    ITPS::FieldPublisher<bool> safetyEnablePub("From:TcpReceiveModule", "SafetyEnable", false); 
    ITPS::FieldPublisher< arma::vec > robotOriginInWorldPub("From:TcpReceiveModule", "RobotOrigin(WorldFrame)", zeroVec2d());
    ITPS::FieldPublisher<bool> initSensorsCmdPub("From:TcpReceiveModule", "re/init sensors", false);
    
    ITPS::FieldSubscriber<bool> ballcapStatusSub("From:BallCaptureModule", "isDribbled");

    

    


    logger.log(Info, "Server Started on Port Number:" + repr(TCP_PORT)
                    + ", Awaiting Remote From:TcpReceiveModule...");

    try 
    {
        ballcapStatusSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        acceptor.accept(socket); // blocks until getting a connection request and accept the connection
    }
    catch(std::exception& e)
    {
        BLogger logger;
        logger.addTag("[TcpReceiveModule.cpp]");
        logger.log(Error, e.what());
        safetyEnablePub.publish(false);
        std::exit(0);
    }

    logger.log(Info, "Connection Established");
    asio::write(socket, asio::buffer("CONNECTION ESTABLISHED\n"));

    // enqueue the backgnd task of this module to the thread pool
    threadPool.execute(boost::bind(&backgndTask, boost::ref(ballcapStatusSub), boost::ref(socket)));


    while(true) { // No delay, blocking-socket-read is used, usually won't use too much CPU resources
        // get first line seperated string from the receiving buffer
        std::istream inputStream(&read_buf);

        try {
            asio::read_until(socket, read_buf, "\n"); 
        }
        catch(std::exception& e) {
            BLogger logger;
            logger.addTag("[TcpReceiveModule.cpp]");
            logger.log(Error, e.what());
            safetyEnablePub.publish(false);

            // To-do: handle disconnect
            while(1) { // has delay (good for reducing high CPU usage)
                delay(1000);
            }
        }
    
        // Tokenize the received input
        std::vector<std::string> tokens;
        std::string tmpStr;
        while(inputStream >> tmpStr) 
        { 
            tokens.push_back(tmpStr); 
        }   

        std::string rtnStr;

        // Processing CommandLines
        if(tokens.size() > 0) {

            // logger.log(Info, tokens[0]);
            // Format: init [x] [y]     where (x,y) is the origin of the robot in the world coordinates
            if(tokens[0] == "init") {
                if(tokens.size() != 3) {
                    rtnStr = "Invalid Arguments";
                }
                else {
                    arma::vec origin = {std::stod(tokens[1]), std::stod(tokens[2])};
                    initSensorsCmdPub.publish(true); // Initialize Sensors
                    robotOriginInWorldPub.publish(origin); // Update Robot's origin point represented in the world frame of reference
                    rtnStr = "Initialized";
                    safetyEnablePub.publish(true);
                }
            }

//            else if(tokens[0] == "anything") {
//                rtnStr = "lmao";
//            }

            else { // invalid command 
                rtnStr = "Invalid Command Received From Remote Side";
            }
        }

        mu.lock();
        asio::write(socket, asio::buffer(rtnStr + "\n"));
        mu.unlock();
    }
 
}

