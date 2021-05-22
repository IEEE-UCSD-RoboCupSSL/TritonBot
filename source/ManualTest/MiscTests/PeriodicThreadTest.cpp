#include "ManualTest/MiscTests/PeriodicThreadTest.hpp"
#include "Misc/PubSubSystem/Module.hpp"
#include <iostream>
#include <string>
#include "Misc/Utility/ClockUtil.hpp"
#include "Config/ModuleFrequencies.hpp"



class HelloWorldModule : public Module {
    void task(ThreadPool& ThreadPool) {
        std::string toPrint = "Hello World";
        auto t0 = std::chrono::steady_clock::now();
        auto t0s = millis();
        std::cout << "This test will quit in 10 seconds" << std::endl;
        while(std::chrono::steady_clock::now() - t0 < std::chrono::seconds(10)) {
            periodic_session([&]() { // expect 1ms error
                std::cout << toPrint << "[" << (millis() - t0s)
                /*DURATION_MILLIS_COUNT(std::chrono::steady_clock::now() - t0)*/ << "]" << std::endl;
                delay(std::chrono::milliseconds(500)); // this delay should not have any effect since 500 < 1000(outer period)
            }, TO_PERIOD(1)); // 1Hz
        }
    }
};




bool PeriodicThreadTest::test(ThreadPool& threadPool) {
    
    HelloWorldModule hwm;
    hwm.run(threadPool);
    
    delay(std::chrono::seconds(10));
    //threadPool.joinAll();
    return true;
}
