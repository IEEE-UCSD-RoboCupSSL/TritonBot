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

BallCaptureModule::BallCaptureModule() : enable_sub("CMD Server", "EnableDribbler"),
                                         ball_pos_sub("GVision Server", "BallPos(BodyFrame)"),
                                         ball_velo_sub("GVision Server", "BallVel(BodyFrame)"),
                                         bot_ang_sub("GVision Server", "BotAng(BodyFrame)"),
                                         bot_ang_velo_sub("GVision Server", "BotAngVel(BodyFrame)"),
                                         bot_pos_sub("GVision Server", "BotPos(BodyFrame)"),
                                         bot_velo_sub("GVision Server", "BotVel(BodyFrame)"),
                                         command_pub("Ball Capture Module", "MotionCMD", default_cmd()),
                                         dribbler_signal_pub("Ball Capture Module", "isOn", dribbler_control),
                                         status_signal_pub("Ball Capture Module", "isDribbled", is_ball_captured),
                                         logger()
                                         
{
    logger.add_tag("BallCapture Module");
}

BallCaptureModule::~BallCaptureModule() = default;


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
        ball_velo_sub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        ball_pos_sub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        bot_velo_sub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        bot_pos_sub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        bot_ang_velo_sub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        bot_ang_sub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        enable_sub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);

    }
    catch(std::exception& e) {
        B_Log logger;
        logger.add_tag("[ball_capture_module.cpp]");
        logger.log(Error, std::string(e.what()));
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


[[noreturn]] void BallCaptureModule::task(ThreadPool& thread_pool) {
    UNUSED(thread_pool);
    init_subscribers();

    while(true){

        if(true){
            logger.log(Info, "Ball displacement (x, y): ( " + std::to_string(ball_pos_sub.latest_msg()(0)) + " , "
                             + std::to_string(ball_pos_sub.latest_msg()(1)) + " )\n");
            logger.log(Info, "Ball velocity (x, y): ( " + std::to_string(ball_velo_sub.latest_msg()(0)) + " , "
                             + std::to_string(ball_velo_sub.latest_msg()(1)) + " )\n");
            logger.log(Info, "Bot displacement (x, y): ( " + std::to_string(bot_pos_sub.latest_msg()(0)) + " , "
                             + std::to_string(bot_pos_sub.latest_msg()(1)) + " )\n");
            logger.log(Info, "Bot velocity (x, y): ( " + std::to_string(bot_velo_sub.latest_msg()(0)) + " , "
                             + std::to_string(bot_velo_sub.latest_msg()(1)) + " )\n");
            logger.log(Info, "Bot angle (O): ( " + std::to_string(bot_ang_sub.latest_msg()) + " )\n");
            logger.log(Info, "Bot ang velocity (w): ( " + std::to_string(bot_ang_velo_sub.latest_msg()) + " )\n");
            delay(100);
        }

        //if(enable_sub.latest_msg()){
        if(true){

//            if(false){
//                logger.log(Info, "[ball capture] dribble status: " + std::to_string(dribble_status));
//                delay(100);
//            }
        }
    }
        
}

bool BallCaptureModule::check_ball_captured(BallEKF_Module::BallData latest_ball_data, MotionEKF_Module::MotionData latest_motion_data){
    double const PI = 3.1415926;
    double const TRANS_VEL_THRESHOLD = 20.0;
    float const ROT_VEL_THRESHOLD = 3.0;

    if(true){
        logger.log(Info, "[ball capture] motion trans_vel (x, y): (" + std::to_string(latest_motion_data.trans_vel(0)) + ", " + std::to_string(latest_motion_data.trans_vel(1)) + ")");
        logger.log(Info, "[ball capture] motion rot_vel: " + std::to_string(latest_motion_data.rotat_vel));
        delay(1000);
    }

    if(latest_motion_data.trans_vel(0) > TRANS_VEL_THRESHOLD || latest_motion_data.trans_vel(1) > TRANS_VEL_THRESHOLD
       || latest_motion_data.rotat_vel > ROT_VEL_THRESHOLD){
        return false;
    }

    double const delta_x = latest_ball_data.disp(0) - latest_motion_data.trans_disp(0);
    double const delta_y = latest_ball_data.disp(1) - latest_motion_data.trans_disp(1);
    double const z = delta_x/delta_y;
    double const angle = std::atan(z) * 180 / PI;
    double orientation = latest_motion_data.rotat_disp < 0 ? 360 + latest_motion_data.rotat_disp : latest_motion_data.rotat_disp;

    if(false){
//        logger.log(Info, "[ball capture] delta_x: " + std::to_string(delta_x));
//        logger.log(Info, "[ball capture] delta_y: " + std::to_string(delta_y));
        logger.log(Info, "[ball capture] ball location (x,y): " + std::to_string(latest_ball_data.disp(0)) + ", " + std::to_string(latest_ball_data.disp(1)) + ")");
        logger.log(Info, "[ball capture] robot location (x,y): " + std::to_string(latest_motion_data.trans_disp(0)) + ", " + std::to_string(latest_motion_data.trans_disp(1)) + ")");
        logger.log(Info, "[ball capture] angle: " + std::to_string(angle));
        logger.log(Info, "[ball capture] orientation: " + std::to_string(orientation));
        delay(1000);
    }

    if(std::abs(angle - orientation) > 30 || std::abs(delta_y) > 100 || std::abs(delta_x) > 100){
        return false;
    }

    return true;
}



