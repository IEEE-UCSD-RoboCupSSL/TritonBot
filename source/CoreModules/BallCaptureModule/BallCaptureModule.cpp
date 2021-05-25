#include "CoreModules/BallCaptureModule/BallCaptureModule.hpp"
#include "Config/ModuleFrequencies.hpp"
#include "CoreModules/DataCmdTypes.hpp"
#include "Misc/Utility/BoostLogger.hpp"
#include "Misc/Utility/Common.hpp"
#include "Config/Config.hpp"


void BallCaptureModule::task(ThreadPool& threadPool) {

    ITPS::FieldSubscriber<bool> enableAutoCaptureSub("From:CommandProcessorModule", "EnableAutoCapture");
    ITPS::FieldSubscriber<BotData> filteredBotDataSub("From:DataProcessorModule", "BotData(BodyFrame)");
    ITPS::FieldSubscriber<BallData> filteredBallDataSub("From:DataProcessorModule", "BallData(BodyFrame)");
    ITPS::FieldSubscriber<bool> isHoldingBallSub("From:DataProcessorModule", "IsHoldingBall");

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
    }
    catch(std::exception& e) {
        BLogger logger;
        logger.addTag("[BallCaptureModule.cpp]");
        logger.log(Error, std::string(e.what()));
        std::exit(0);
    }

    logger(Info) << "\033[0;32m Initialized \033[0m";

    while(true) {
        periodic_session([&](){
            auto isHoldingBall = isHoldingBallSub.getMsg();
            auto ballData = filteredBallDataSub.getMsg();
            auto botData = filteredBotDataSub.getMsg();
            if(isHoldingBall) {
                ballCapStatusPub.publish(true);
            } else {
                ballCapStatusPub.publish(false);
            } 

            if(enableAutoCaptureSub.getMsg()) {
                auto mCmd = config.botConfig->autoBallCaptureSolution(isHoldingBall, ballData, botData);
                
                ballAutoCapMotionCommandPub.publish(mCmd); 
            }
        }, TO_PERIOD(BALL_CAPTURE_FREQUENCY));
    }

}

