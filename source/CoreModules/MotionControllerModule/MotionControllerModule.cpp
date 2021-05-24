#include "CoreModules/MotionControllerModule/MotionControllerModule.hpp"
#include "Config/ModuleFrequencies.hpp"
#include "CoreModules/DataCmdTypes.hpp"
#include "Misc/Utility/BoostLogger.hpp"
#include "Misc/Utility/Common.hpp"
#include "Config/Config.hpp"
#include "CoreModules/Conversion.hpp"
#include "CoreModules/MotionControllerModule/PIDAlgorithm.hpp"

void MotionControllerModule::task(ThreadPool& threadPool) {
    ITPS::FieldSubscriber<bool> dribblerCommandSub("From:CommandProcessorModule", "dribblerSwitch");
    ITPS::FieldSubscriber<arma::vec2> kickerSetPointSub("From:CommandProcessorModule", "KickerSetPoint(On/Off)");
    ITPS::FieldSubscriber<ControlInput> controlInputSub("From:CommandProcessorModule", "MotionControlInput");
    ITPS::FieldSubscriber<bool> safetyEnableSub("From:TcpReceiveModule", "SafetyEnable"); 
    try {
        dribblerCommandSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        kickerSetPointSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        controlInputSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        safetyEnableSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
    }
    catch(std::exception& e) {
        BLogger logger;
        logger.addTag("[MotionControllerModule.cpp]");
        logger.log(Error, std::string(e.what()));
        std::exit(0);
    }

    while(true) {
        while(safetyEnableSub.getMsg()) {
            periodic_session([&](){


            }, TO_PERIOD(MOTION_CONTROLLER_FREQUENCY));
        }
    }
}
