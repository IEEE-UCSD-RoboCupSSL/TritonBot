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

static RemoteGeometry default_cmd;

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


    // Current hard-coding IP address and Port #
    std::string ip = "";
    unsigned int port = 0;

    asio::io_service io_service;
    boost::asio::ip::tcp::endpoint ep(boost::asio::ip::address::from_string(ip), port);
    boost::asio::ip::tcp::socket socket(io_service);
    socket.open(boost::asio::ip::tcp::v4());
    socket.connect(ep);

    logger.log(Info, "Successsfully connected to " + ip + " port " + repr(port) + "\n");

    // // publisher to publish data sent from vfirm: [vfirm socket] => [firm_data_pub] => [EKF module]
    // ITPS::Publisher<bool> precise_kick_pub("ConnectionServer", "PreciseKickData");
    // ITPS::Publisher<bool> ball_capture_pub("ConnectionServer", "BallCaptureData");