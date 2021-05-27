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

    delay(1000);

    initSensorsCmdPub.publish(true);
    for(float pwr = 0.0; true; pwr+=0.01) {
        if(pwr >= 100.00) pwr = 0.0;
        ControlOutput co;
        co.vx = pwr;
        co.vy = -pwr;
        co.omega = pwr / 2.0;
        controlOutputPub.publish(co);
        arma::vec2 ko = {pwr / 2.0, pwr / 10.0};
        kickerSetPointPub.publish(ko);
        if(pwr >= 50.00) dribblerCommandPub.publish(true);
        else dribblerCommandPub.publish(false);

        auto data = mcuSensorDataSub.getMsg();
        std::cout << "Enc[" << data.encCnt << "] ImuAcc[" << data.imuAcc
                    << "] ImuTheta[" << data.imuTheta << "] ImuOmega[" << data.imuOmega
                    << "] IsHoldingBall[" << data.isHoldingBall << "]" << std::endl;
    }
    
    threadPool.joinAll();
    return true;
}
