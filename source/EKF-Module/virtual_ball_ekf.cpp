#include "EKF-Module/virtual_ball_ekf.hpp"
#include "PubSubSystem/thread_pool.hpp"
#include "Utility/boost_logger.hpp"
#include "Config/config.hpp"
#include "ProtoGenerated/messages_robocup_ssl_wrapper.pb.h"
#include "ProtoGenerated/messages_robocup_ssl_detection.pb.h"
#include "ProtoGenerated/messages_robocup_ssl_geometry.pb.h"
#include "Utility/common.hpp"

const bool is_blue_team_side = true;

using namespace boost;
using namespace boost::asio;
using namespace boost::asio::ip;

VirtualBallEKF::VirtualBallEKF() : BallEKF_Module() {}


VirtualBallEKF::~VirtualBallEKF() {}


void VirtualBallEKF::task(ThreadPool& thread_pool) {
    B_Log logger;
    logger.add_tag("PseudoBallEKF Module");

    logger(Info) << "\033[0;32m Thread Started \033[0m";

    init_subscribers();
    logger(Info) << "\033[0;32m Initialized \033[0m";

    io_service io_service;
    udp::endpoint grsim_endpoint(address::from_string(GRSIM_VISION_IP), GRSIM_VISION_PORT);
    udp::socket socket(io_service);

    socket.open(grsim_endpoint.protocol());
    socket.set_option(udp::socket::reuse_address(true));
    socket.bind(grsim_endpoint);
    socket.set_option(ip::multicast::join_group(grsim_endpoint.address()));

    size_t num_received;
    std::string packet_received;
    boost::array<char, UDP_RBUF_SIZE> receive_buffer;

    BallEKF::BallData ball_data;
    ball_data.loc = zero_vec_2d();
    ball_data.vel = zero_vec_2d();

    size_t num_bytes_received;
    std::string packet_string;
    SSL_WrapperPacket packet;
    google::protobuf::RepeatedPtrField<SSL_DetectionBall> balls;




    while(1) {


        num_bytes_received = socket.receive_from(asio::buffer(receive_buffer), grsim_endpoint);
        packet_string = std::string(receive_buffer.begin(), 
                                    receive_buffer.begin() + num_bytes_received);

        packet.ParseFromString(packet_string);

        balls = packet.detection().balls();
        

        std::cout << balls.size() << std::endl;

        if(balls.size() == 1) {
            auto ball = balls[0];

            // grsim's x & y are reversed due to diff view perspective
            if(is_blue_team_side) {
                ball_data.loc(0) = ball.y();
                ball_data.loc(1) = -ball.x();
            }
            else {
                ball_data.loc(0) = -ball.y();
                ball_data.loc(1) = ball.x();
            }
            publish_ball_data(ball_data);
        
        }
        
    }
}