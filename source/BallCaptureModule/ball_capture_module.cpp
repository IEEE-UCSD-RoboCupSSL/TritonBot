#include "BallCaptureModule/ball_capture_module.hpp"
#include "Config/config.hpp"
#include "Utility/boost_logger.hpp"
#include "Utility/common.hpp"



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
                                         command_pub("CMD Server", "MotionCMD", default_cmd()),
                                         dribbler_signal_pub("CMD Server", "isDone", dribbler_control),
                                         status_signal_pub("CMD Server", "isDone", is_ball_captured)
                                         
{}

BallCaptureModule::~BallCaptureModule() {} 


void BallCaptureModule::init_subscribers() {
    try {
        // ...
        enable_sub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        ball_data_sub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
    }
    catch(std::exception& e) {
        B_Log logger;
        logger.add_tag("[ball_capture_module.cpp]");
        logger.log(Error, e.what());
        std::exit(0);
    }

}


void BallCaptureModule::task(ThreadPool& thread_pool) {
    UNUSED(thread_pool);
    init_subscribers();


    
}