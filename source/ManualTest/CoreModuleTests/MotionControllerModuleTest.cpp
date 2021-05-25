#include "ManualTest/CoreModuleTests/MotionControllerModuleTest.hpp"
#include "CoreModules/DataCmdTypes.hpp"
#include "DataFusion/BallDataFusion.hpp"
#include "DataFusion/BotDataFusion.hpp"
#include "CoreModules/MotionControllerModule/MotionControllerModule.hpp"
#include "Misc/Utility/Common.hpp"
#include "Misc/Utility/ClockUtil.hpp"
#include "Misc/Utility/BoostLogger.hpp"
#include "Config/Config.hpp"
#include <cassert>

bool MotionControllerModuleTest::test(ThreadPool& threadPool) {
   
    MotionControllerModule motionControllerModule(config);
    motionControllerModule.run(threadPool);



    /*** Mock Subscribers setup ***/    
    ITPS::FieldSubscriber<ControlOutput> controlOutputSub("From:MotionControllerModule", "MotionControlOutput");



    /*** Mock Publishers setup ***/
    ITPS::FieldPublisher<ControlInput> controlInputPub("From:CommandProcessorModule", "MotionControlInput", defaultControlInput());
    ITPS::FieldPublisher<bool> safetyEnablePub("From:TcpReceiveModule", "SafetyEnable", true); 
    ITPS::FieldPublisher<BotData> filteredBotDataPub("From:DataProcessorModule", "BotData(BodyFrame)", defaultBotData());
    

    try {
        controlOutputSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
    }
    catch(std::exception& e) {
        BLogger logger;
        logger.addTag("[MotionControllerModuleTest.cpp]");
        logger.log(Error, std::string(e.what()));
        std::exit(0);
    }

    delay(1000);
    std::cout << "pos-pid-consts: " << config.botConfig->posPidConsts.toArmaVec3() << std::endl;
    std::cout << "angle-pid-consts: " << config.botConfig->anglePidConsts.toArmaVec3() << std::endl;

    auto ci = defaultControlInput(); 
    // auto btd = defaultBotData(); // default bot pos = (0, 0)
    while(true) {
        std::cout << "Enter Mode (TDRD, TDRV, TVRD, or TVRV)" << std::endl;
        std::string mode;
        std::cin >> mode;
        if(mode == "TDRD") {
            ci.translationalSetPoint.type = SetPointType::position;
            ci.rotationalSetPoint.type = SetPointType::position;
        }
        if(mode == "TDRV") {
            ci.translationalSetPoint.type = SetPointType::position;
            ci.rotationalSetPoint.type = SetPointType::velocity;
        }

        if(mode == "TVRD") {
            ci.translationalSetPoint.type = SetPointType::velocity;
            ci.rotationalSetPoint.type = SetPointType::position;
        }

        if(mode == "TVRV") {
            ci.translationalSetPoint.type = SetPointType::velocity;
            ci.rotationalSetPoint.type = SetPointType::velocity;
        }
        std::cout << "Enter Control Input (x, y, omega)" << std::endl;
        std::cin >> ci.translationalSetPoint.value(0) >> ci.translationalSetPoint.value(1) >> ci.rotationalSetPoint.value;
        controlInputPub.publish(ci);
        delay(10);
        auto out = controlOutputSub.getMsg();
        std::cout << "Output: " << out.toArmaVec3() << std::endl;
    }


    threadPool.joinAll();
    return true;
}