#include "EKF-Module/virtual_ball_ekf.hpp"
#include "PubSubSystem/thread_pool.hpp"
#include "Utility/boost_logger.hpp"
#include "Config/config.hpp"
#include "ProtoGenerated/messages_robocup_ssl_wrapper.pb.h"
#include "ProtoGenerated/messages_robocup_ssl_detection.pb.h"
#include "ProtoGenerated/messages_robocup_ssl_geometry.pb.h"
#include "Utility/common.hpp"

const bool is_blue_team_side = true;
const int vel_sample_period_ms = 50; // 50 ms 
const double vel_max_thresh = 10000.00; // 10000 mm/s == 10 m/s (threshold for the norm of vel vector)

using namespace boost;
using namespace boost::asio;
using namespace boost::asio::ip;

VirtualBallEKF::VirtualBallEKF() : BallEKF_Module() {}


VirtualBallEKF::~VirtualBallEKF() {}


void VirtualBallEKF::task(ThreadPool& thread_pool) {
    logger.add_tag("PseudoBallEKF Module");
    logger(Info) << "\033[0;32m Thread Started \033[0m";
    init_subscribers();
    logger(Info) << "\033[0;32m Initialized \033[0m";

    grsim_endpoint = boost::shared_ptr<udp::endpoint>(
        new udp::endpoint(address::from_string(GRSIM_VISION_IP), GRSIM_VISION_PORT)
    );
    this->socket = boost::shared_ptr<udp::socket>(new udp::socket(io_service));
    this->timer = boost::shared_ptr<deadline_timer>(new deadline_timer(io_service));

    socket->open(grsim_endpoint->protocol());
    socket->set_option(udp::socket::reuse_address(true));
    socket->bind(*grsim_endpoint);
    socket->set_option(ip::multicast::join_group(grsim_endpoint->address()));

    // default async task that keeps looping in the background
    io_service.post(boost::bind(&VirtualBallEKF::loop, this)); // .post means postpone, meaning that this task has lower priority in the task queue
    
    // timed periodic task
    timer->expires_from_now(posix_time::millisec(vel_sample_period_ms));
    timer->async_wait(boost::bind(&VirtualBallEKF::velocity_calc_timer_task, this)); 

    io_service.run();
}

void VirtualBallEKF::velocity_calc_timer_task() {
    arma::vec vel = ((ball_data.disp - prev_disp) / vel_sample_period_ms) * 1000.00; // unit: mm/s
    if(arma::norm(vel) < vel_max_thresh) {
        ball_data.vel = vel;
    }
    prev_disp = ball_data.disp;

    // recursive call at the next time point
    timer->expires_from_now(posix_time::millisec(vel_sample_period_ms));
    timer->async_wait(boost::bind(&VirtualBallEKF::velocity_calc_timer_task, this));
}


void VirtualBallEKF::loop() {
    boost::array<char, UDP_RBUF_SIZE> receive_buffer;
    
    size_t num_bytes_received;
    std::string packet_string;
    SSL_WrapperPacket packet;
    google::protobuf::RepeatedPtrField<SSL_DetectionBall> balls;

    num_bytes_received = socket->receive_from(asio::buffer(receive_buffer), *grsim_endpoint);
    packet_string = std::string(receive_buffer.begin(), 
                                receive_buffer.begin() + num_bytes_received);

    packet.ParseFromString(packet_string);

    balls = packet.detection().balls();
    

    if(balls.size() == 1) {
        auto ball = balls[0];

        // grsim's x & y are reversed due to diff view perspective
        if(is_blue_team_side) {
            ball_data.disp = {-ball.y(), ball.x()};
        }
        else {
            ball_data.disp = {ball.y(), -ball.x()};
        }
        publish_ball_data(ball_data);
    
    }

    // Recursive call to enqueue loop func as an dft async task
    io_service.post(boost::bind(&VirtualBallEKF::loop, this));
}