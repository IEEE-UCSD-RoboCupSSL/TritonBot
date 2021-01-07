#include "BallCaptureModule/ball_capture_module.hpp"
#include "Config/config.hpp"
#include "Utility/boost_logger.hpp"
#include "Utility/common.hpp"
#include "Utility/systime.hpp"


// Note: rotational data are all in world frame
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
                                         bot_data_sub("MotionEKF", "MotionData"),
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
        bot_data_sub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
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

        if(false){
            logger.log(Info, "Ball displacement (x, y): ( " + std::to_string(ball_pos_sub.latest_msg()(0)) + " , "
                             + std::to_string(ball_pos_sub.latest_msg()(1)) + " )\n");
//            logger.log(Info, "Ball velocity (x, y): ( " + std::to_string(ball_velo_sub.latest_msg()(0)) + " , "
//                             + std::to_string(ball_velo_sub.latest_msg()(1)) + " )\n");
//            logger.log(Info, "Bot displacement (x, y): ( " + std::to_string(bot_data_sub.latest_msg().trans_disp(0)) + " , "
//                             + std::to_string(bot_data_sub.latest_msg().trans_disp(1)) + " )\n");
//            logger.log(Info, "Delta displacement (x, y): ( " + std::to_string(ball_pos_sub.latest_msg()(0) - bot_data_sub.latest_msg().trans_disp(0)) +
//            ", " + std::to_string(ball_pos_sub.latest_msg()(1) - bot_data_sub.latest_msg().trans_disp(1)) + " )\n");
//            logger.log(Info, "Bot velocity (x, y): ( " + std::to_string(bot_data_sub.latest_msg().trans_vel(0)) + " , "
//                             + std::to_string(bot_data_sub.latest_msg().trans_vel(1)) + " )\n");
//            logger.log(Info, "Bot angle (O): ( " + std::to_string(bot_data_sub.latest_msg().rotat_disp) + " )\n");
//            logger.log(Info, "Bot ang velocity (w): ( " + std::to_string(bot_data_sub.latest_msg().rotat_vel) + " )\n");
            delay(100);
        }

        if(enable_sub.latest_msg()){
        //if(true){

            if(false){
                logger.log(Info, "[ball capture] dribble status: " + std::to_string(check_ball_captured_V(ball_pos_sub.latest_msg(), bot_data_sub.latest_msg())));
                delay(100);
            }
            arma::vec ball_pos = ball_pos_sub.latest_msg();
            MotionEKF_Module::MotionData latest_motion_data = bot_data_sub.latest_msg();

            double delta_x = ball_pos(0) - latest_motion_data.trans_disp(0);
            double delta_y = ball_pos(1) - latest_motion_data.trans_disp(1);
            double angle = calc_angle(delta_y, delta_x);

            Motion::MotionCMD command;

            if(angle < 20.0 && angle > -20.0){
                command.mode = Motion::CTRL_Mode::TDRD;
                command.ref_frame = Motion::ReferenceFrame::BodyFrame;
                command.setpoint_3d = {ball_pos(0), ball_pos(1), angle + bot_data_sub.latest_msg().rotat_disp};
                command_pub.publish(command);
            }
            else{
                command.mode = Motion::CTRL_Mode::TVRD;
                command.ref_frame = Motion::ReferenceFrame::BodyFrame;
                command.setpoint_3d = {0, 0, angle + bot_data_sub.latest_msg().rotat_disp};
                command_pub.publish(command);
            }

            if(true){
                logger.log(Info, "[ball capture] capture command (x, y, O) " + std::to_string(command.mode) + " : (" + std::to_string(delta_x) + ", " + std::to_string(delta_y) +
                ", " + std::to_string(angle + bot_data_sub.latest_msg().rotat_disp) + ")\n");
            }

            dribbler_signal_pub.publish(check_ball_captured_V(ball_pos, latest_motion_data));
        }
    }
        
}

bool BallCaptureModule::check_ball_captured_V(arma::vec ball_pos, MotionEKF_Module::MotionData latest_motion_data){
    double const PI = 3.1415926;
    double const X_TRESHOLD = 40.0;
    double const Y_TRESHOLD = 20.0;
    double const DRIBBLER_OFFSET = 100.0;

    double const delta_x = ball_pos(0) - latest_motion_data.trans_disp(0);
    double const delta_y = ball_pos(1) - latest_motion_data.trans_disp(1);
//    double const z = delta_x/delta_y;
//    double const angle = std::atan(z) * 180 / PI;

    if(std::abs(delta_x) < X_TRESHOLD && delta_y < DRIBBLER_OFFSET + Y_TRESHOLD && delta_y > DRIBBLER_OFFSET - Y_TRESHOLD){
        return true;
    }

    return false;
}

double BallCaptureModule::calc_angle(double delta_y, double delta_x){
    double angle;

    if(delta_x < 0.0001 && delta_x > -0.0001){
        if(delta_y > 0){
            return 0;
        }
        else{
            return 179.9;
        }

    }

    // first quadrant
    if(delta_y >= 0 && delta_x >= 0){
        angle = std::atan(delta_y/delta_x) * 180.0 / 3.1415926 - 90;
    }
    // second quadrant
    else if(delta_y >= 0 && delta_x < 0) {
        angle = 90 - std::atan(delta_y/-delta_x) * 180.0 / 3.1415926;
    }
    // fourth quadrant
    else if(delta_y < 0 && delta_x >= 0){
        angle = std::atan(delta_y/delta_x) * 180.0 / 3.1415926 - 90;
    }
    // third quadrant
    else{
        angle = 90 - std::atan(delta_y/-delta_x) * 180.0 / 3.1415926;
    }

    return angle;

}



bool BallCaptureModule::is_this_module_enabled() {
    return false;
}



