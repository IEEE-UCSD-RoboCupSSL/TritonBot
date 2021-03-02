#include "PeriphModules/RemoteServers/ConServerModule.hpp"

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

using namespace boost;

boost::mutex mu;


static void backgnd_task(ITPS::NonBlockingSubscriber<bool>& ballcap_status_sub, 
                         asio::ip::tcp::socket& socket) {
    bool prev_ballcap_status = true; // deliberately set it true to have a extra socket send at the begining
    while(1) { // has delay (good for reducing high CPU usage)

        delay(500); // this delay is important now because EKF is not yet implemented, 
                    // pseudo ekf doesn't handle the issue of botLoc & ballLoc data being received at different frequency 

        std::string send_str;
        bool ballcap_status;
        ballcap_status = ballcap_status_sub.latest_msg(); 
        if(ballcap_status != prev_ballcap_status) {
            if(ballcap_status) {
                send_str = "BallOnHold";
            }
            else {
                send_str = "BallOffHold";
            }
            mu.lock();
            asio::write(socket, asio::buffer(send_str + "\n"));
            mu.unlock();
        }
        prev_ballcap_status = ballcap_status;
        
    }

}


// Implementation of task to be run on this thread
void ConnectionServer::task(ThreadPool& thread_pool) {
    UNUSED(thread_pool); // no child thread is needed in a async scheme

    B_Log logger;
    logger.add_tag("Connection Server Module");
    logger(Info) << "\033[0;32m Thread Started \033[0m";


    int field_length = 0;
    int field_width = 0;
    int goal_depth = 0;
    int goal_width = 0;

    asio::io_service io_service;
    asio::ip::tcp::endpoint endpoint_to_listen(asio::ip::tcp::v4(), CONN_SERVER_PORT); 
    asio::ip::tcp::acceptor acceptor(io_service, endpoint_to_listen);
    asio::ip::tcp::socket socket(io_service);
    asio::streambuf read_buf;
    std::string write_buf;

    ITPS::NonBlockingPublisher<bool> safety_enable_pub("AI Connection", "SafetyEnable", true); // To-do: change it back to false after testing
    ITPS::NonBlockingPublisher< arma::vec > robot_origin_w_pub("ConnectionInit", "RobotOrigin(WorldFrame)", zero_vec_2d());
    ITPS::NonBlockingPublisher<bool> init_sensors_pub("vfirm-client", "re/init sensors", false);
    ITPS::NonBlockingSubscriber<bool> ballcap_status_sub("Ball Capture Module", "isDribbled");

    

    


    logger.log(Info, "Server Started on Port Number:" + repr(CONN_SERVER_PORT) 
                    + ", Awaiting Remote AI Connection...");

    try 
    {
        ballcap_status_sub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        acceptor.accept(socket); // blocks until getting a connection request and accept the connection
    }
    catch(std::exception& e)
    {
        B_Log logger;
        logger.add_tag("[connection_server_module.cpp]");
        logger.log(Error, e.what());
        safety_enable_pub.publish(false);
        std::exit(0);
    }

    logger.log(Info, "Connection Established");
    asio::write(socket, asio::buffer("CONNECTION ESTABLISHED\n"));

    // enqueue the backgnd task of this module to the thread pool
    thread_pool.execute(boost::bind(&backgnd_task, boost::ref(ballcap_status_sub), boost::ref(socket)));


    while(1) { // No delay, blocking-socket-read is used, usually won't use too much CPU resources
        // get first line seperated string from the receiving buffer
        std::istream input_stream(&read_buf);

        try {
            asio::read_until(socket, read_buf, "\n"); 
        }
        catch(std::exception& e) {
            B_Log logger;
            logger.add_tag("[connection_server_module.cpp]");
            logger.log(Error, e.what());
            safety_enable_pub.publish(false);

            // To-do: handle disconnect
            while(1) { // has delay (good for reducing high CPU usage)
                delay(1000);
            }
        }
    
        // Tokenize the received input
        std::vector<std::string> tokens;
        std::string tmp_str;
        while(input_stream >> tmp_str) 
        { 
            tokens.push_back(tmp_str); 
        }   

        std::string rtn_str;

        // Processing CommandLines
        if(tokens.size() > 0) {

            // logger.log(Info, tokens[0]);
            // Format: init [x] [y]     where (x,y) is the origin of the robot in the world coordinates
            if(tokens[0] == "init") {
                if(tokens.size() != 3) {
                    rtn_str = "Invalid Arguments";
                }
                else {
                    arma::vec origin = {std::stod(tokens[1]), std::stod(tokens[2])};
                    init_sensors_pub.publish(true); // Initialize Sensors
                    robot_origin_w_pub.publish(origin); // Update Robot's origin point represented in the world frame of reference
                    rtn_str = "Initialized";
                    safety_enable_pub.publish(true);
                }
            }

            else if(tokens[0] == "anything") {
                rtn_str = "bazinga";
            }

            else { // invalid command 
                rtn_str = "Invalid Command Received From Remote Side";
            }
        }

        mu.lock();
        asio::write(socket, asio::buffer(rtn_str + "\n"));
        mu.unlock();
    }
 
}

