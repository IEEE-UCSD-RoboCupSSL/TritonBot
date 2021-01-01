#include <ProtoGenerated/messages_robocup_ssl_wrapper.pb.h>
#include "EKF-Module/virtual_motion_ekf.hpp"
#include "PubSubSystem/thread_pool.hpp"
#include "ProtoGenerated/vFirmware_API.pb.h"
#include "Utility/boost_logger.hpp"
#include "Config/config.hpp"

using namespace boost;
using namespace boost::asio;
using namespace boost::asio::ip;

const bool is_blue_team_side = true;
const int vel_sample_period_ms = 50; // 50 ms
const double vel_max_thresh = 10000.00; // 1000

VirtualMotionEKF::VirtualMotionEKF() : MotionEKF_Module() {}


VirtualMotionEKF::~VirtualMotionEKF() = default;


[[noreturn]] void VirtualMotionEKF::task(ThreadPool& thread_pool){
    B_Log logger;
    logger.add_tag("PseudoMotionEKF Module");

    logger(Info) << "\033[0;32m Thread Started \033[0m";

    init_subscribers();
    logger(Info) << "\033[0;32m Initialized \033[0m";

    VF_Data vf_data;
    MotionEKF::MotionData m_data;

    while(true) {
        // Motion data is return in non-global frame!
        // The actual
        vf_data = get_firmware_data();

        if(false){
            logger.log(Info, "[virtual_motion_ekf] trans_disp_x_y: (" + std::to_string(vf_data.translational_displacement().x()) + ", " + std::to_string(vf_data.translational_displacement().y()) + ")");
            logger.log(Info, "[virtual_motion_ekf] trans_vel_x_y: (" + std::to_string(vf_data.translational_velocity().x()) + ", " + std::to_string(vf_data.translational_velocity().y()) + ")");
            logger.log(Info, "[virtual_motion_ekf] rotate_disp: (" + std::to_string(vf_data.rotational_displacement()) + ")");
            logger.log(Info, "[virtual_motion_ekf] rotate_vel: (" + std::to_string(vf_data.rotational_velocity()) + ")");
        }
        
        m_data.trans_disp = {vf_data.translational_displacement().x(), 
                             vf_data.translational_displacement().y()};

        m_data.trans_vel = {vf_data.translational_velocity().x(),
                            vf_data.translational_velocity().y()};

        m_data.rotat_disp = vf_data.rotational_displacement();
        m_data.rotat_vel = vf_data.rotational_velocity();

        publish_motion_data(m_data);
        
    }
}

void VirtualMotionEKF::getPositionData() {
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
    io_service.post(boost::bind(&VirtualMotionEKF::loop, this)); // .post means postpone, meaning that this task has lower priority in the task queue

    // timed periodic task
    timer->expires_from_now(posix_time::millisec(vel_sample_period_ms));
    timer->async_wait(boost::bind(&VirtualMotionEKF::velocity_calc_timer_task, this));
}

void VirtualMotionEKF::loop() {
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
            motion_data.trans_disp = {-ball.y(), ball.x()};
        }
        else {
            motion_data.trans_disp = {ball.y(), -ball.x()};
        }

    }

    // Recursive call to enqueue loop func as an dft async task
    io_service.post(boost::bind(&VirtualMotionEKF::loop, this));

}

void VirtualMotionEKF::velocity_calc_timer_task() {
    arma::vec vel = ((motion_data.trans_disp - prev_disp) / vel_sample_period_ms) * 1000.00; // unit: mm/s
    if(arma::norm(vel) < vel_max_thresh) {
        motion_data.trans_vel = vel;
    }
    prev_disp = motion_data.trans_disp;

    // recursive call at the next time point
    timer->expires_from_now(posix_time::millisec(vel_sample_period_ms));
    timer->async_wait(boost::bind(&VirtualMotionEKF::velocity_calc_timer_task, this));
}