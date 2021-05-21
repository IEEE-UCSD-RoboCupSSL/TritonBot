#include "ManualTest/MiscTests/PubSubTest.hpp"
#include "Misc/PubSubSystem/Module.hpp"
#include <iostream>
#include <string>
#include "Misc/Utility/Systime.hpp"
#include "Misc/PubSubSystem/PubSub.hpp"





using namespace ITPS;

bool PubSubTest::test(ThreadPool& threadPool) {
    
    FieldPubSubPair<int> psPair("PubSubTest", "xxx", 0);

    std::cout << "PubSubTest..." << std::endl;


    threadPool.execute([&](){
        int i = 0;
        auto t0 = CHRONO_NOW;
        while(CHRONO_NOW - t0 < std::chrono::milliseconds(3000)) {
            auto t = CHRONO_NOW;
            psPair.pub->publish(++i);
            std::this_thread::sleep_until(t + std::chrono::milliseconds(100));
        }
    });

    threadPool.execute([&](){
        auto t0 = CHRONO_NOW;
        while(CHRONO_NOW - t0 < std::chrono::milliseconds(3000)) {
            std::cout << psPair.sub->latest_msg() << std::endl;
            delay(std::chrono::milliseconds(50));     
        }
    });

    delay(5000);
    std::cout << "End of Test" << std::endl;
    return true;
}
