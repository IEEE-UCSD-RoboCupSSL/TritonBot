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

#define __MAGIC asio::ip::tcp::socket& socket, \
                std::string& write_buf, \
                asio::streambuf& read_buf, \
                B_Log& logger, \
                const system::error_code& error

#define __MAGIC_BIND(FUNC_NAME) boost::bind(&FUNC_NAME, \
                                            boost::ref(socket), \
                                            boost::ref(write_buf), \
                                            boost::ref(read_buf), \
                                            boost::ref(logger), \
                                            asio::placeholders::error)

//======================== Local callback function declaration =========================//
static void on_connected(__MAGIC);

static void on_geodata_received(__MAGIC);

static void on_status_return(__MAGIC);
//======================== End of local callback declaration ===========================//


//======================== Local callback function implementation =========================//
static void on_connected(__MAGIC)
{
    if(error)
    {
        logger.log(Error, error.message());
        return;
    }

    logger.log(Info, "Accepted socket request from: " + socket.remote_endpoint().address().to_string() + "\n");

    write_buf.clear();
    write_buf = "CONNECTION ESTABLISHED\n";
    boost::asio::write(socket, asio::buffer(write_buf));
    
    asio::async_read_until(socket, read_buf, "\n", __MAGIC_BIND(on_geodata_received));
}

static void on_geodata_received(__MAGIC)
{
    if(error)
    {
        logger.log(Error, error.message());
        return;
    }

    RemoteGeometry data;
    std::istream input_stream(&read_buf); // check me
    std::string received;

    received = std::string(std::istreambuf_iterator<char>(input_stream), {});            
    data.ParseFromString(received);

    logger.log( Info, "(field length, field width) " + repr(data.field_length()) + ' ' + repr(data.field_width()));
    logger.log( Info, "(goal_depth, goal_width)" + repr(data.goal_depth()) + ' ' + repr(data.goal_width()));

    // TODO: since the following modules are not here yet... Uncomment in the future
    // bool precise_kick_status = precise_kick_sub.latest_msg();
    // bool ball_capture_status = ball_capture_sub.latest_msg();

    write_buf.clear();
    write_buf = "GEOMETRY RECEIVED\n";
    boost::asio::write(socket, asio::buffer(write_buf));

    
    while(1) {
        std::cout << "bageyalu" << std::endl;
    }
}



//======================== End of local callback implementation ===========================//


// Implementation of task to be run on this thread
void ConnectionServer::task(ThreadPool& thread_pool) {
    UNUSED(thread_pool); // no child thread is needed in a async scheme

    B_Log logger;
    logger.add_tag("Connection Server Module");
    logger(Info) << "\033[0;32m Thread Started \033[0m";

    RemoteGeometry commands;

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
        acceptor.async_accept(socket, __MAGIC_BIND(on_connected)); // blocking func
    }
    catch(std::exception& e)
    {
        logger.log(Error, "[Exception]" + std::string(e.what()));
    }

    io_service.run();
    
}