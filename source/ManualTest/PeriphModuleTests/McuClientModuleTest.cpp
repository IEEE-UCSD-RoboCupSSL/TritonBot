#include "ManualTest/PeriphModuleTests/McuClientModuleTest.hpp"
#include "Misc/PubSubSystem/Module.hpp"
#include <iostream>
#include <string>
#include "Misc/Utility/ClockUtil.hpp"
#include "Config/ModuleFrequencies.hpp"
#include "PeriphModules/McuClientModule/McuClientModule.hpp"
#include "Misc/Utility/Common.hpp"
#include "Config/Config.hpp"
#include "CoreModules/DataCmdTypes.hpp"

bool McuClientModuleTest::test(ThreadPool& threadPool) {
    McuClientModule mcuClientModule(config);
    
    // Run the modules
    mcuClientModule.run(threadPool);

    // Mock
    ITPS::FieldPublisher<ControlOutput> controlOutputPub("From:MotionControllerModule", "MotionControlOutput", defaultControlOutput());
    ITPS::FieldPublisher<bool> dribblerCommandPub("From:CommandProcessorModule", "dribblerSwitch", false);
    ITPS::FieldPublisher<arma::vec2> kickerSetPointPub("From:CommandProcessorModule", "KickerSetPoint(On/Off)", zeroVec2d());
    ITPS::FieldPublisher<bool> initSensorsCmdPub("From:TcpReceiveModule", "re/init sensors", false);
   
    ITPS::FieldSubscriber<McuSensorData> mcuSensorDataSub("From:McuClientModule", "McuSensorData(BodyFrame)");
  
    

    try {
        mcuSensorDataSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
    } catch(std::exception& e) {
        BLogger logger;
        logger.addTag("[McuClientModuleTest.cpp]");
        logger.log(Error, std::string(e.what()));
        std::exit(0);
    }

    
    threadPool.joinAll();
    return true;
}
