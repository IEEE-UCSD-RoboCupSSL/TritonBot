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