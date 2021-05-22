#include "CoreModules/CommandProcessorModule/CommandProcessorModule.hpp"
#include "Config/ModuleFrequencies.hpp"
#include "CoreModules/DataCmdTypes.hpp"
#include "Misc/Utility/BoostLogger.hpp"
#include "Misc/Utility/Common.hpp"
#include "Config/Config.hpp"
#include "CoreModules/Conversion.hpp"




void CommandProcessorModule::task(ThreadPool& threadPool) {

    // ITPS::FieldSubscriber<Command> receivedCommandSub("From:UdpReceiveModule", "Command");
    


    /*** Subscriber setup ***/
    ITPS::FieldSubscriber< MotionCommand > motionCmdSub("From:UdpReceiveModule", "MotionCommand");
    ITPS::FieldSubscriber< bool > enAutoCapSub("From:UdpReceiveModule", "EnableAutoCap");
    ITPS::FieldSubscriber<arma::vec> kickerSetPointSub("From:UdpReceiveModule", "KickingSetPoint");
    ITPS::FieldSubscriber<arma::vec> robotOriginInWorldSub("From:TcpReceiveModule", "RobotOrigin(WorldFrame)");
    ITPS::FieldSubscriber<BotData> botDataSub("MotionEKF", "BotProcessedData");
    ITPS::FieldSubscriber< MotionCommand > ballCapMotionCmdSub("From:BallCaptureModule", "MotionCommand");



    try {
        ballCapMotionCmdSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        robotOriginInWorldSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        botDataSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        motionCmdSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        enAutoCapSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        kickerSetPointSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
    }
    catch(std::exception& e) {
        BLogger logger;
        logger.addTag("[CommandProcessorModule.cpp]");
        logger.log(Error, std::string(e.what()));
        std::exit(0);
    }




    while(true) {
        periodic_session([&](){

            


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



*/