#include "EKF-Module/virtual_ball_ekf.hpp"
#include "PubSubSystem/thread_pool.hpp"
#include "Utility/boost_logger.hpp"
#include "Config/config.hpp"
#include "ProtoGenerated/messages_robocup_ssl_wrapper.pb.h"
#include "ProtoGenerated/messages_robocup_ssl_detection.pb.h"
#include "ProtoGenerated/messages_robocup_ssl_geometry.pb.h"
#include "Utility/common.hpp"


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


    BallData ball_data;

    while(true) {
        ball_data.disp = get_ball_loc();
        ball_data.vel = get_ball_vel();

        // logger.log(Info, "<" + repr(ball_data.disp(0)) + ", " + repr(ball_data.disp(1)) + ">");
        publish_ball_data(ball_data);
    }
}
