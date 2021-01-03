#include "EKF-Module/ball_ekf_module.hpp"
#include "Config/config.hpp"
#include "Utility/boost_logger.hpp"
#include "Utility/common.hpp"




static BallEKF::BallData dft_bd() {
    BallEKF::BallData rtn;
    rtn.disp = zero_vec_2d();
    rtn.vel = zero_vec_2d();
    return rtn;
} 


BallEKF_Module::BallEKF_Module() : ball_data_pub("BallEKF", "BallData", dft_bd()),
                                   ball_loc_sub("GVision Server", "BallPos(BodyFrame)"),
                                   ball_vel_sub("GVision Server", "BallVel(BodyFrame)")
{}

BallEKF_Module::~BallEKF_Module() {} 


void BallEKF_Module::init_subscribers() {
    try {
        ball_loc_sub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        ball_vel_sub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
    }
    catch(std::exception& e) {
        B_Log logger;
        logger.add_tag("[ball_ekf_module.cpp]");
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

