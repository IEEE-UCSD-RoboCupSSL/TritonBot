#pragma once

#include <iostream>
#include <unordered_map>
#include <string>
#include "Config/Config.hpp"
#include "Misc/PubSubSystem/ThreadPool.hpp"
#include "ManualTest/ManualTest.hpp"
#include "ManualTest/MiscTests/PeriodicThreadTest.hpp"
#include "ManualTest/MiscTests/PubSubTest.hpp"
#include "ManualTest/PeriphModuleTests/TcpReceiveModuleTest.hpp"
#include "ManualTest/PeriphModuleTests/UdpReceiveModuleTest.hpp"
#include "ManualTest/CoreModuleTests/ConversionTest.hpp"
#include "ManualTest/CoreModuleTests/DataProcessorModuleTest.hpp"
#include "ManualTest/CoreModuleTests/CommandProcessorModuleTest.hpp"
#include "ManualTest/CoreModuleTests/MotionControllerModuleTest.hpp"
#include "ManualTest/DEPRECATED_vftest.hpp"


class TestRunner {
public:
    TestRunner(Config cfg) : config(cfg) {
        testsMap["periodic-thread"] = new PeriodicThreadTest();
        testsMap["pubsub"] = new PubSubTest();
        testsMap["tcp-recei"] = new TcpReceiveModuleTest(config);
        testsMap["udp-recei"] = new UdpReceiveModuleTest(config);
        testsMap["conversion"] = new ConversionTest();
        testsMap["data-proc"] = new DataProcessorModuleTest(config);
        testsMap["cmd-proc"] = new CommandProcessorModuleTest(config);
        testsMap["motion-ctrl"] = new MotionControllerModuleTest(config); 
        testsMap["vf"] = new VFTest(config);
    }
    ~TestRunner() {
        for(auto it = testsMap.begin(); it != testsMap.end(); it++) {
            delete it->second;
        }
    }
    void run(ThreadPool& threadPool);

protected:
    ManualTest* findTest(std::string testName);
    std::unordered_map<std::string, ManualTest*> testsMap;  

private:
    void printAllAvailableTests();
    Config config;
};


