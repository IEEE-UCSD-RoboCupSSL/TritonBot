#include "CoreModules/CommandProcessorModule/CommandProcessorModule.hpp"
#include "Config/ModuleFrequencies.hpp"
#include "CoreModules/DataCmdTypes.hpp"
#include "Misc/Utility/BoostLogger.hpp"
#include "Misc/Utility/Common.hpp"
#include "Config/Config.hpp"

void CommandProcessorModule::task(ThreadPool& threadPool) {


    /*** Subscriber setup ***/
    ITPS::FieldSubscriber<arma::vec> robotOriginInWorldSub("From:TcpReceiveModule", "RobotOrigin(WorldFrame)");
    ITPS::FieldSubscriber<BotData> botDataSub("MotionEKF", "BotProcessedData");
    ITPS::FieldSubscriber< MotionCommand > ballCapMotionCmdSub("From:BallCaptureModule", "MotionCommand");


    try {
        ballCapMotionCmdSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        robotOriginInWorldSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        botDataSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
    }
    catch(std::exception& e) {
        BLogger logger;
        logger.addTag("[CommandProcessorModule.cpp]");
        logger.log(Error, std::string(e.what()));
        std::exit(0);
    }






    while(true) {
        periodic_session([](){

        }, TO_PERIOD(COMMAND_PROCESSOR_FREQUENCY));
    }
}

/* 

if(!udpData.commanddata().enable_ball_auto_capture()) {
                           }
            else {
                // Listening to internal AutoCapture module's commands
                enAutoCapPub.publish(true);
                mCmd = ballCapMotionCmdSub.latest_msg();
                motionCmdPub.publish(mCmd);
            }

*/

/*

            // reference frame transformation math
            arma::vec botOrigin = robotOriginInWorldSub.latest_msg();
            float botOrien = botDataSub.latest_msg().ang;
            botPos = transform(botOrigin, botOrien, botPos);
            botVel = transform(botOrigin, botOrien, botVel);
            ballPos = transform(botOrigin, botOrien, ballPos);
            ballVel = transform(botOrigin, botOrien, ballVel);

// for explaination of the math, check motion_module.cpp
static arma::vec transform(arma::vec origin, float orien, arma::vec point2d) {
    arma::mat A = wtb_homo_transform(origin, orien); // world to body homogeneous transformation
    arma::vec p_homo_w = {point2d(0), point2d(1), 1}; // homogeneous point end with a 1 (vector end with a 0)
    arma::vec p_homo_b = A * p_homo_w; // apply transformation to get the same point represented in the body frame
    // if division factor is approx. eq to zero
    if(std::fabs(p_homo_b(2)) < 0.000001) {
        p_homo_b(2) = 0.000001;
    }
    // update setpoint to the setpoint in robot's perspective (cartesean coordinate)
    arma::vec p_cart_b = {p_homo_b(0)/p_homo_b(2), p_homo_b(1)/p_homo_b(2)}; // the division is to divide the scaling factor, according to rules of homogeneous coord systems
    return p_cart_b;
}

*/