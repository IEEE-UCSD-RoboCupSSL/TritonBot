#include "BallCaptureModule/ball_capture_module.hpp"
#include "Config/config.hpp"
#include "Utility/boost_logger.hpp"
#include "Utility/common.hpp"
#include "Utility/systime.hpp"



static Motion::MotionCMD default_cmd() {
    Motion::MotionCMD dft_cmd;
    dft_cmd.setpoint_3d = {0, 0, 0};
    dft_cmd.mode = Motion::CTRL_Mode::TVRV;
    dft_cmd.ref_frame = Motion::ReferenceFrame::BodyFrame;
    return dft_cmd;
}

static bool dribbler_control = false;

static bool is_ball_captured = false;

BallCaptureModule::BallCaptureModule() : enable_sub("CMD Server", "EnableAutoBallCapture"),
                                         ball_data_sub("BallEKF", "BallData"),
                                         motion_data_sub("MotionEKF", "MotionData"),
                                         command_pub("Ball Capture Module", "MotionCMD", default_cmd()),
                                         dribbler_signal_pub("Ball Capture Module", "isTurnedOn", dribbler_control),
                                         status_signal_pub("Ball Capture Module", "isDribbled", is_ball_captured),
                                         logger()
                                         
{
    logger.add_tag("BallCapture Module");
}

BallCaptureModule::~BallCaptureModule() {} 


void BallCaptureModule::init_subscribers() {
    // try {
    //     // ...
    //     enable_sub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
    // }
    // catch(std::exception& e) {
    //     B_Log logger;
    //     logger.add_tag("[ball_capture_module.cpp]");
    //     logger.log(Error, std::string(e.what()) + " | EnableAutoBallCapture");
    //     std::exit(0);
    // }

    try {
        // ...
        ball_data_sub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
    }
    catch(std::exception& e) {
        B_Log logger;
        logger.add_tag("[ball_capture_module.cpp]");
        logger.log(Error, std::string(e.what()) + " | BallEKF");
        std::exit(0);
    }

    // try {
    //     // ...
    //     motion_data_sub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
    // }
    // catch(std::exception& e) {
    //     B_Log logger;
    //     logger.add_tag("[ball_capture_module.cpp]");
    //     logger.log(Error, std::string(e.what()) + " | MotionEKF");
    //     std::exit(0);
    // }

}


void BallCaptureModule::task(ThreadPool& thread_pool) {
    UNUSED(thread_pool);
    init_subscribers();
    
    while(1){
        if(1){
            // MotionEKF_Module::MotionData latest_motion_data = motion_data_sub.latest_msg(); // Robot data is in robot frame
            BallEKF_Module::BallData latest_ball_data = ball_data_sub.latest_msg(); // Ball data is in world frame
            logger.log(Info, "Ball velocity (x, y): ( " + std::to_string(latest_ball_data.vel(0)) + " , " 
                                                        + std::to_string(latest_ball_data.vel(1)) + " )\n");
            delay(1000);
        }
    }
        
}



