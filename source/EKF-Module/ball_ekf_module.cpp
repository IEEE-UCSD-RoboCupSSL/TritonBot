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


BallEKF_Module::BallEKF_Module() : ball_data_pub("BallEKF", "BallData", dft_bd())
{}

BallEKF_Module::~BallEKF_Module() {} 


void BallEKF_Module::init_subscribers() {
    try {
        // add ssl vision subscriber later
    }
    catch(std::exception& e) {
        B_Log logger;
        logger.add_tag("[ball_ekf_module.cpp]");
        logger.log(Error, e.what());
        while(1);
    }

}


void BallEKF_Module::publish_ball_data(BallData data) {
    ball_data_pub.publish(data);
}