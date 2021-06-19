#include "CoreModules/BallCaptureModule/BallCaptureModule.hpp"
#include "Config/ModuleFrequencies.hpp"
#include "CoreModules/DataCmdTypes.hpp"
#include "Misc/Utility/BoostLogger.hpp"
#include "Misc/Utility/Common.hpp"
#include "Config/Config.hpp"


[[noreturn]] void BallCaptureModule::task(ThreadPool& threadPool) {

    ITPS::FieldSubscriber<bool> enableAutoCaptureSub("From:CommandProcessorModule", "EnableAutoCapture");
    ITPS::FieldSubscriber<BotData> filteredBotDataSub("From:DataProcessorModule", "BotData(BodyFrame)");
    ITPS::FieldSubscriber<BallData> filteredBallDataSub("From:DataProcessorModule", "BallData(BodyFrame)");
    ITPS::FieldSubscriber<bool> isHoldingBallSub("From:DataProcessorModule", "IsHoldingBall");
    ITPS::FieldSubscriber<arma::vec2> robotOriginInWorldSub("From:TcpReceiveModule", "RobotOrigin(WorldFrame)");

    ITPS::FieldPublisher<MotionCommand> ballAutoCapMotionCommandPub("From:BallAutoCaptureModule", "MotionCommand(BodyFrame)", defaultMotionCommand());
    ITPS::FieldPublisher<bool> ballCapStatusPub("From:BallCaptureModule", "isDribbled", false);

    
    BLogger logger;
    logger.addTag("BallCaptureModule");
    logger(Info) << "\033[0;32m Thread Started \033[0m";

    try {
        enableAutoCaptureSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        filteredBallDataSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        filteredBotDataSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        isHoldingBallSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        robotOriginInWorldSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
    }
    catch(std::exception& e) {
        BLogger logger;
        logger.addTag("[BallCaptureModule.cpp]");
        logger.log(Error, std::string(e.what()));
        std::exit(0);
    }

    logger(Info) << "\033[0;32m Initialized \033[0m";
    bool prevEnAutoCap = false;
    double angle = 0.00;

    while(true) {
        periodic_session([&](){
            auto isHoldingBall = isHoldingBallSub.getMsg();
            auto ballData = filteredBallDataSub.getMsg();
            auto botData = filteredBotDataSub.getMsg();
            arma::vec2 robotOrigin = robotOriginInWorldSub.getMsg();

            if(isHoldingBall) {
                ballCapStatusPub.publish(true);
            } else {
                ballCapStatusPub.publish(false);
            } 
            bool enAutoCap = enableAutoCaptureSub.getMsg();

            if(enAutoCap) {
                if(!prevEnAutoCap) {
                    prevEnAutoCap = true;
                    angle = botData.ang;
                }

                auto mCmd = config.botConfig->autoBallCaptureSolution(isHoldingBall, ballData, botData, 0.5, angle,
                                                                      robotOrigin);
                
                ballAutoCapMotionCommandPub.publish(mCmd); 
            } else {
                prevEnAutoCap = false;
            }

        }, TO_PERIOD(BALL_CAPTURE_FREQUENCY));
    }

}

