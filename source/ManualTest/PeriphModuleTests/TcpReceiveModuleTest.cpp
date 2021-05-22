#include "ManualTest/PeriphModuleTests/TcpReceiveModuleTest.hpp"
#include "Misc/PubSubSystem/Module.hpp"
#include <iostream>
#include <string>
#include "Misc/Utility/Systime.hpp"
#include "Config/ModuleFrequencies.hpp"
#include "PeriphModules/RemoteServers/TcpReceiveModule.hpp"


bool TcpReceiveModuleTest::test(ThreadPool& threadPool) {
    
    /* TCP receive module already ran by TestRunner */


    // Mock
    ITPS::FieldPublisher<bool> ballcapStatusPub("From:BallCaptureModule", "isDribbled", false);

    threadPool.execute([&](){
        bool tmp = false;
        while(true) {
            tmp = tmp ? false : true;
            ballcapStatusPub.publish(tmp);
            delay(std::chrono::seconds(1));
        }
    });



    threadPool.joinAll();
    return true;
}
