#include "ConnectionServerModule/connection_server.hpp"

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

//======================== Local callback function declaration =========================//
static void on_connected(asio::ip::tcp::socket& socket,
                            std::string& write_buf,
                            asio::streambuf& read_buf, 
                            ITPS::Subscriber<bool>& precise_kick_sub,
                            ITPS::Subscriber<bool>& ball_capture_sub,
                            B_Log& logger,  
                            const system::error_code& error);

static void on_data_received(asio::ip::tcp::socket& socket,
                            std::string& write_buf,
                            asio::streambuf& read_buf, 
                            ITPS::Subscriber<bool>& precise_kick_sub,
                            ITPS::Subscriber<bool>& ball_capture_sub,
                            B_Log& logger,  
                            const system::error_code& error);

static void on_status_return(asio::ip::tcp::socket& socket,
                            std::string& write_buf,
                            asio::streambuf& read_buf, 
                            ITPS::Subscriber<bool>& precise_kick_sub,
                            ITPS::Subscriber<bool>& ball_capture_sub,
                            B_Log& logger,  
                            const system::error_code& error);
//======================== End of local callback declaration ===========================//


//======================== Local callback function implementation =========================//
static void on_connected(asio::ip::tcp::socket& socket,
                            std::string& write_buf,
                            asio::streambuf& read_buf, 
                            ITPS::Subscriber<bool>& precise_kick_sub,
                            ITPS::Subscriber<bool>& ball_capture_sub,
                            B_Log& logger,  
                            const system::error_code& error)
{
    if(error)
    {
        logger.log(Error, error.message());
        return;
    }

    logger.log(Info, "Accepted socket request from: " + socket.remote_endpoint().address().to_string() + "\n");
    
    asio::async_read_until(socket, read_buf, "\n", boost::bind(&on_data_received, 
                                            boost::ref(socket),
                                            boost::ref(write_buf), 
                                            boost::ref(read_buf), 
                                            boost::ref(precise_kick_sub), 
                                            boost::ref(ball_capture_sub), 
                                            boost::ref(logger), 
                                            asio::placeholders::error));
}

static void on_data_received(asio::ip::tcp::socket& socket,
                            std::string& write_buf,
                            asio::streambuf& read_buf, 
                            ITPS::Subscriber<bool>& precise_kick_sub,
                            ITPS::Subscriber<bool>& ball_capture_sub,
                            B_Log& logger,  
                            const system::error_code& error)
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

    logger.log( Debug, "(field length, field width) " + repr(data.field_length()) + ' ' + repr(data.field_width()));
    logger.log( Debug, "(goal_depth, goal_width)" + repr(data.goal_depth()) + ' ' + repr(data.goal_width()));

    // TODO: since the following modules are not here yet... Uncomment in the future
    // bool precise_kick_status = precise_kick_sub.latest_msg();
    // bool ball_capture_status = ball_capture_sub.latest_msg();

    write_buf.clear();
    write_buf = "CONNECTION ESTABLISHED";
    
    asio::async_write(socket, asio::buffer(write_buf), boost::bind(&on_status_return, 
                                            boost::ref(socket),
                                            boost::ref(write_buf), 
                                            boost::ref(read_buf), 
                                            boost::ref(precise_kick_sub), 
                                            boost::ref(ball_capture_sub), 
                                            boost::ref(logger), 
                                            asio::placeholders::error));

}

static void on_status_return(asio::ip::tcp::socket& socket,
                            std::string& write_buf,
                            asio::streambuf& read_buf, 
                            ITPS::Subscriber<bool>& precise_kick_sub,
                            ITPS::Subscriber<bool>& ball_capture_sub,
                            B_Log& logger,  
                            const system::error_code& error)
{
        if(error)
        {
            logger.log(Error, error.message());
            return;
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
    asio::ip::tcp::endpoint endpoint_to_listen(asio::ip::tcp::v4(), 6666);
    asio::ip::tcp::acceptor acceptor(io_service, endpoint_to_listen);
    asio::ip::tcp::socket socket(io_service);
    asio::streambuf read_buf;
    std::string write_buf;

    ITPS::Subscriber<bool> precise_kick_sub("ConnectionServer", "PreciseKickData");
    ITPS::Subscriber<bool> ball_capture_sub("ConnectionServer", "BallCaptureData");

    // TODO: since the following modules are not here yet... Uncomment in the future
    // while(!precise_kick_sub.subscribe());
    // while(!ball_capture_sub.subscribe());
    logger.log(Info, "Server started, port number: 6666. Awaiting Remote Station connection... \n");

    try 
    {
        acceptor.async_accept(socket, boost::bind(&on_connected, 
                                            boost::ref(socket),
                                            boost::ref(write_buf), 
                                            boost::ref(read_buf), 
                                            boost::ref(precise_kick_sub), 
                                            boost::ref(ball_capture_sub), 
                                            boost::ref(logger), 
                                            asio::placeholders::error)); // blocking func
    }
    catch(std::exception& e)
    {
        logger.log(Error, "[Exception]" + std::string(e.what()));
    }

    io_service.run();
    
}
 
