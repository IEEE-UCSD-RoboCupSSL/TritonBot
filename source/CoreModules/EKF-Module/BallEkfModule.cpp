#include "CoreModules/EKF-Module/BallEkfModule.hpp"
#include "Config/Config.hpp"
#include "Misc/Utility/BoostLogger.hpp"
#include "Misc/Utility/Common.hpp"
#include "Misc/PubSubSystem/ThreadPool.hpp"
#include "Misc/Utility/BoostLogger.hpp"
#include "Config/Config.hpp"
#include "ProtoGenerated/messages_robocup_ssl_wrapper.pb.h"
#include "ProtoGenerated/messages_robocup_ssl_detection.pb.h"
#include "ProtoGenerated/messages_robocup_ssl_geometry.pb.h"
#include "Misc/Utility/Common.hpp"
#include "Misc/Utility/Systime.hpp"


using namespace boost;
using namespace boost::asio;
using namespace boost::asio::ip;


/*   */
static BallEKF::BallData dft_bd() {
    BallEKF::BallData rtn;
    rtn.disp = zeroVec2d();
    rtn.vel = zeroVec2d();
    return rtn;
} 


BallEKF_Module::BallEKF_Module() : ball_data_pub("BallEKF", "BallData", dft_bd()),
                                   ball_loc_sub("From:UdpReceiveModule", "BallPos(BodyFrame)"),
                                   ball_vel_sub("From:UdpReceiveModule", "BallVel(BodyFrame)")
{}

BallEKF_Module::~BallEKF_Module() {} 


void BallEKF_Module::init_subscribers() {
    try {
        ball_loc_sub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        ball_vel_sub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
    }
    catch(std::exception& e) {
        BLogger logger;
        logger.addTag("[ball_ekf_module.cpp]");
        logger.log(Error, e.what());
        std::exit(0);
    }

}


void BallEKF_Module::publish_ball_data(BallData data) {
    ball_data_pub.publish(data);
}

arma::vec BallEKF_Module::get_ball_loc() {
    return ball_loc_sub.latest_msg();
}

arma::vec BallEKF_Module::get_ball_vel() {
    return ball_vel_sub.latest_msg();
}

/*   */

/*   */

VirtualBallEKF::VirtualBallEKF() : BallEKF_Module() {}


VirtualBallEKF::~VirtualBallEKF() {}


void VirtualBallEKF::task(ThreadPool& threadPool) {
    BLogger logger;
    logger.addTag("PseudoBallEKF Module");
    logger(Info) << "\033[0;32m Thread Started \033[0m";
    init_subscribers();
    logger(Info) << "\033[0;32m Initialized \033[0m";


    BallData ball_data;

    

    while(true) { // has delay (good for reducing high CPU usage)
        ball_data.disp = get_ball_loc();
        ball_data.vel = get_ball_vel();

        // logger.log(Info, "<" + repr(ball_data.disp(0)) + ", " + repr(ball_data.disp(1)) + ">");
        publish_ball_data(ball_data);

        delay(1);
    }
}

/*   */