#include "ManualTest/PeriphModuleTests/TcpReceiveModuleTest.hpp"
#include "Misc/PubSubSystem/Module.hpp"
#include <iostream>
#include <string>
#include "Misc/Utility/ClockUtil.hpp"
#include "Config/ModuleFrequencies.hpp"
#include "PeriphModules/RemoteServers/TcpReceiveModule.hpp"
#include "ManualTest/TestRunner.hpp"
#include <cassert>


bool TcpReceiveModuleTest::test(ThreadPool& threadPool) {
    
    /* TCP receive module already ran by TestRunner */

    ITPS::FieldSubscriber<bool> safetyEnableSub("From:TcpReceiveModule", "SafetyEnable"); 
    try {
        safetyEnableSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
    }
    catch(std::exception& e) {
        BLogger logger;
        logger.addTag("[TcpReceiveModuleTest.cpp]");
        logger.log(Error, std::string(e.what()));
        std::exit(0);
    }


    threadPool.execute([&](){
        bool tmp = false;
        auto t0 = CHRONO_NOW;
        while(CHRONO_NOW - t0 < std::chrono::seconds(7)) {
            tmp = tmp ? false : true;
            ballcapStatusPubMock->publish(tmp);
            delay(std::chrono::seconds(1));
        }
    });

    std::cout << "This test will quit in 8 seconds" << std::endl;

    assert(safetyEnableSub.getMsg() == true);


    delay(8000);

    // threadPool.joinAll();
    return true;
}
