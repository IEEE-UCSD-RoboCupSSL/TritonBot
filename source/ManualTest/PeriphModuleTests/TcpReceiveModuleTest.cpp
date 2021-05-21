#include "ManualTest/PeriphModuleTests/TcpReceiveModuleTest.hpp"
#include "Misc/PubSubSystem/Module.hpp"
#include <iostream>
#include <string>
#include "Misc/Utility/Systime.hpp"
#include "Config/ModuleFrequencies.hpp"
#include "PeriphModules/RemoteServers/TcpReceiveModule.hpp"


bool TcpReceiveModuleTest::test(ThreadPool& threadPool) {
    
    std::unique_ptr<TcpReceiveModule> tcpReceiveModule(new TcpReceiveModule());

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


    // Run the module
    tcpReceiveModule->run(threadPool);

    threadPool.joinAll();
    return true;
}
