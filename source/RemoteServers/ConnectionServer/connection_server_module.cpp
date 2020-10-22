#include "RemoteServers/ConnectionServer/connection_server_module.hpp"

#include <string>
#include <thread>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include "Utility/systime.hpp"
#include "PubSubSystem/thread_pool.hpp"
#include "ProtoGenerated/RemoteAPI.pb.h"
#include "Utility/boost_logger.hpp"
#include "Config/config.hpp"
#include "Utility/common.hpp"

using namespace boost;


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

    // ITPS::BlockingSubscriber<bool> precise_kick_sub("ConnectionServer", "KickerStatusRtn");
    // ITPS::BlockingSubscriber<bool> ball_capture_sub("ConnectionServer", "BallCaptureStatusRtn");

    // TODO: since the following modules are not here yet... Uncomment in the future
    // while(!precise_kick_sub.subscribe());
    // while(!ball_capture_sub.subscribe());

    logger.log(Info, "Server Started on Port Number:" + repr(CONN_SERVER_PORT) 
                    + ", Awaiting Remote AI Connection...");

    try 
    {
        acceptor.accept(socket); // blocks until getting a connection request and accept the connection
    }
    catch(std::exception& e)
    {
        logger.log(Error, "[Exception]" + std::string(e.what()));
        while(1);
    }



    while(1) {
        // get first line seperated string from the receiving buffer
        std::istream input_stream(&read_buf);
        asio::read_until(socket, read_buf, "\n"); 
        std::string received = std::string(std::istreambuf_iterator<char>(input_stream), {});
        std::stringstream ss(received);

        // Tokenize the received string
        std::vector<std::string> tokens;
        std::string tmp_str;
        while(getline(ss, tmp_str, ' ')) 
        { 
            tokens.push_back(tmp_str); 
        }   

        // Processing CommandLines
        if(tokens.size() > 0) {
            if(tokens[0] == "init") {

            }

            else if(tokens[0] == "") {

            }

            else { // invalid command 
                logger.log(Warning, "Invalid Command Received From Remote Side");
            }
        }
    }




    
}
