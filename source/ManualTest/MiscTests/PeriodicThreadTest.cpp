#include "ManualTest/MiscTests/PeriodicThreadTest.hpp"
#include "Misc/PubSubSystem/Module.hpp"
#include <iostream>
#include <string>
#include "Misc/Utility/Systime.hpp"



class HelloWorldModule : public Module {
    void task(ThreadPool& ThreadPool) {
        std::string toPrint = "Hello World";
        auto t0 = std::chrono::steady_clock::now();
        while(true) {
            periodic_session([&]() { // expect 1ms error
                std::cout << toPrint << "[" << 
                DURATION_MILLIS_COUNT(std::chrono::steady_clock::now() - t0) << "]" << std::endl;
                delay(std::chrono::milliseconds(500)); // this delay should not have any effect since 500 < 1000(outer period)
            }, std::chrono::milliseconds(1000)); 
        }
    }
};




bool PeriodicThreadTest::test(ThreadPool& threadPool) {
    
    HelloWorldModule hwm;
    hwm.run(threadPool);
    threadPool.joinAll();
    return true;
}
