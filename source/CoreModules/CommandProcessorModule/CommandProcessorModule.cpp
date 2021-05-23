#include "CoreModules/CommandProcessorModule/CommandProcessorModule.hpp"
#include "Config/ModuleFrequencies.hpp"
#include "CoreModules/DataCmdTypes.hpp"
#include "Misc/Utility/BoostLogger.hpp"
#include "Misc/Utility/Common.hpp"
#include "Config/Config.hpp"
#include "CoreModules/Conversion.hpp"




void CommandProcessorModule::task(ThreadPool& threadPool) {
    /*** Publisher setup ***/
    ITPS::FieldPublisher<bool> dribblerCommand("From:CommandProcessorModule", "DribblerSwitch", false);
    ITPS::FieldPublisher<arma::vec> kickerSetPoint("From:CommandProcessorModule", "KickerSetPoint(On/Off)", zeroVec2d());


    /*** Subscriber setup ***/
    ITPS::FieldSubscriber<Command> receivedCommandSub("From:UdpReceiveModule", "Command");
    ITPS::FieldSubscriber<arma::vec> robotOriginInWorldSub("From:TcpReceiveModule", "RobotOrigin(WorldFrame)");
    ITPS::FieldSubscriber<BotData> filteredBotDataSub("From:DataProcessorModule", "BotData(BodyFrame)");
    ITPS::FieldSubscriber<BallData> filteredBallDataSub("From:DataProcessorModule", "BallData(BodyFrame)");
    
    try {
        receivedCommandSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        robotOriginInWorldSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        filteredBallDataSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        filteredBotDataSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
    }
    catch(std::exception& e) {
        BLogger logger;
        logger.addTag("[CommandProcessorModule.cpp]");
        logger.log(Error, std::string(e.what()));
        std::exit(0);
    }




    while(true) {
        periodic_session([&](){
            //dribblerCommand.publish();

            


        }, TO_PERIOD(COMMAND_PROCESSOR_FREQUENCY));
    }
}

/*
bool isBallCloseEnoughToBot(arma::vec ballPos, BotData botData) {
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
}*/




/* 

if(!udpData.commanddata().enable_ball_auto_capture()) {
                           }
            else {
                // Listening to internal AutoCapture module's commands
                enAutoCapPub.publish(true);
                mCmd = ballCapMotionCmdSub.getMsg();
                motionCmdPub.publish(mCmd);
            }

*/

/*

            // reference frame transformation math
            arma::vec botOrigin = robotOriginInWorldSub.getMsg();
            float botOrien = botDataSub.getMsg().ang;
            botPos = transform(botOrigin, botOrien, botPos);
            botVel = transform(botOrigin, botOrien, botVel);
            ballPos = transform(botOrigin, botOrien, ballPos);
            ballVel = transform(botOrigin, botOrien, ballVel);



*/