#include "Misc/Utility/BoostLogger.hpp"
#include "Misc/Utility/Common.hpp"
#include "Misc/Utility/Systime.hpp"
#include "CoreModules/BallCaptureModule/BallCaptureModule.hpp"
#include "CoreModules/EKF-Module/BallEkfModule.hpp"
#include "Config/Config.hpp"

#define AUTOCAP_DECISION_THRESHOLD 80
#define DRIBBLER_ENABLE_DIS 300.0

// Note: rotational data are all in world frame
static Motion::MotionCMD defaultCmd() {
    Motion::MotionCMD dft_cmd;
    dft_cmd.setpoint3d = {0, 0, 0};
    dft_cmd.mode = Motion::CTRL_Mode::TVRV;
    dft_cmd.refFrame = Motion::ReferenceFrame::BodyFrame;
    return dft_cmd;
}


BallCaptureModule::BallCaptureModule() : enable_sub("From:UdpReceiveModule", "EnableAutoCap"),
                                         ball_data_sub("BallEKF", "BallData"),
                                         bot_data_sub("MotionEKF", "BotProcessedData"),
                                         command_pub("From:BallCaptureModule", "MotionCommand", defaultCmd()),
                                         ballcap_status_pub("From:BallCaptureModule", "isDribbled", false),
                                         drib_enable_pub("BallCapture", "EnableDribbler", false),
                                         logger() {
    logger.addTag("BallCapture Module");
}

BallCaptureModule::~BallCaptureModule() = default;


void BallCaptureModule::init_subscribers() {

    try {
        ball_data_sub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        bot_data_sub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        enable_sub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);

    }
    catch (std::exception &e) {
        BLogger logger;
        logger.addTag("[ball_capture_module.cpp]");
        logger.log(Error, std::string(e.what()));
        std::exit(0);
    }

}


// [[noreturn]] 
void BallCaptureModule::task(ThreadPool &threadPool) {
    UNUSED(threadPool);
    init_subscribers();
    int count = 0;
    int total = 0;

    while (true) { // has delay (good for reducing high CPU usage)

//         if(false){
//             logger.log(Info, "Ball displacement (x, y): ( " + std::to_string(ball_pos_sub.latest_msg()(0)) + " , "
//                              + std::to_string(ball_pos_sub.latest_msg()(1)) + " )\n");
// //            logger.log(Info, "Ball velocity (x, y): ( " + std::to_string(ball_velo_sub.latest_msg()(0)) + " , "
// //                             + std::to_string(ball_velo_sub.latest_msg()(1)) + " )\n");
// //            logger.log(Info, "Bot displacement (x, y): ( " + std::to_string(bot_data_sub.latest_msg().pos(0)) + " , "
// //                             + std::to_string(bot_data_sub.latest_msg().pos(1)) + " )\n");
// //            logger.log(Info, "Delta displacement (x, y): ( " + std::to_string(ball_pos_sub.latest_msg()(0) - bot_data_sub.latest_msg().pos(0)) +
// //            ", " + std::to_string(ball_pos_sub.latest_msg()(1) - bot_data_sub.latest_msg().pos(1)) + " )\n");
// //            logger.log(Info, "Bot velocity (x, y): ( " + std::to_string(bot_data_sub.latest_msg().vel(0)) + " , "
// //                             + std::to_string(bot_data_sub.latest_msg().vel(1)) + " )\n");
// //            logger.log(Info, "Bot angle (O): ( " + std::to_string(bot_data_sub.latest_msg().ang) + " )\n");
// //            logger.log(Info, "Bot ang velocity (w): ( " + std::to_string(bot_data_sub.latest_msg().angVel) + " )\n");
//             delay(100);
//         }

        bool averageResult = false;

        arma::vec ball_pos = ball_data_sub.latest_msg().disp;
        MotionEKF_Module::BotData latest_motion_data = bot_data_sub.latest_msg();

        if (check_close_enough(ball_pos, latest_motion_data)) {
            drib_enable_pub.publish(true);
        } else {
            drib_enable_pub.publish(false);
        }


        if (check_ball_captured_V(ball_pos, latest_motion_data)) {
            count++;
        }

        total++;

        if (total >= 100) {
            if (count >= AUTOCAP_DECISION_THRESHOLD) {
                averageResult = true;
                ballcap_status_pub.publish(true);
            } else {
                averageResult = false;
                ballcap_status_pub.publish(false);
            }

            count = 0;
            total = 0;
        }

        if (enable_sub.latest_msg()) {

            // if(false){
            //     logger.log(Info, "[ball capture] dribble status: " + std::to_string(check_ball_captured_V(ball_data_sub.latest_msg().disp, bot_data_sub.latest_msg())));
            //     delay(100);
            // }


            double delta_x = ball_pos(0) - latest_motion_data.pos(0);
            double delta_y = ball_pos(1) - latest_motion_data.pos(1);
            double angle = calc_angle(delta_y, delta_x);

            Motion::MotionCMD command;


            if (!averageResult) {
                command.mode = Motion::CTRL_Mode::TDRD;
                command.refFrame = Motion::ReferenceFrame::BodyFrame;
                command.setpoint3d = {ball_pos(0), ball_pos(1), angle + latest_motion_data.ang};
                command_pub.publish(command);
            } else {
                command.mode = Motion::CTRL_Mode::TVRD;
                command.refFrame = Motion::ReferenceFrame::BodyFrame;
                command.setpoint3d = {0, 5.00, angle + latest_motion_data.ang};
                command_pub.publish(command);
            }

            // if(true){
            //     logger.log(Info, "[ball capture] capture command (x, y, O) " + std::to_string(command.mode) + " : (" + std::to_string(delta_x) + ", " + std::to_string(delta_y) +
            //     ", " + std::to_string(angle + bot_data_sub.latest_msg().ang) + ")\n");
            // }

        }

        delay(1);
    }

}

bool BallCaptureModule::check_close_enough(arma::vec ball_pos, MotionEKF_Module::BotData latest_motion_data) {
    double const PI = 3.1415926;
    double const X_TRESHOLD = 200.0;
    double const Y_TRESHOLD = 200.0;
    double const DRIBBLER_OFFSET = 105.0;

    double delta_x = ball_pos(0) - latest_motion_data.pos(0);
    double delta_y = ball_pos(1) - latest_motion_data.pos(1);

    if (std::abs(delta_x) < X_TRESHOLD
        && delta_y < DRIBBLER_OFFSET + Y_TRESHOLD
        && delta_y > DRIBBLER_OFFSET - Y_TRESHOLD) {
        return true;
    }

    return false;
}

bool BallCaptureModule::check_ball_captured_V(arma::vec ball_pos, MotionEKF_Module::BotData latest_motion_data) {
    double const PI = 3.1415926;
    double const X_TRESHOLD = 80.0;
    double const Y_TRESHOLD = 20.0;
    double const DRIBBLER_OFFSET = 105.0;

    double delta_x = ball_pos(0) - latest_motion_data.pos(0);
    double delta_y = ball_pos(1) - latest_motion_data.pos(1);

    if (std::abs(delta_x) < X_TRESHOLD
        && delta_y < DRIBBLER_OFFSET + Y_TRESHOLD
        && delta_y > DRIBBLER_OFFSET - Y_TRESHOLD) {
        return true;
    }

    return false;
}

double BallCaptureModule::calc_angle(double delta_y, double delta_x) {
    double angle;

    if (delta_x < 0.0001 && delta_x > -0.0001) {
        if (delta_y > 0) {
            return 0;
        } else {
            return 179.9;
        }

    }

    // first quadrant
    if (delta_y >= 0 && delta_x >= 0) {
        angle = std::atan(delta_y / delta_x) * 180.0 / 3.1415926 - 90;
    }
        // second quadrant
    else if (delta_y >= 0 && delta_x < 0) {
        angle = 90 - std::atan(delta_y / -delta_x) * 180.0 / 3.1415926;
    }
        // fourth quadrant
    else if (delta_y < 0 && delta_x >= 0) {
        angle = std::atan(delta_y / delta_x) * 180.0 / 3.1415926 - 90;
    }
        // third quadrant
    else {
        angle = 90 - std::atan(delta_y / -delta_x) * 180.0 / 3.1415926;
    }

    return angle;

}






