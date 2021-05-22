#pragma once

#include <iostream>
#include <unordered_map>
#include <string>
#include "Misc/PubSubSystem/ThreadPool.hpp"
#include "ManualTest/ManualTest.hpp"
#include "ManualTest/MiscTests/PeriodicThreadTest.hpp"
#include "ManualTest/MiscTests/PubSubTest.hpp"
#include "ManualTest/PeriphModuleTests/TcpReceiveModuleTest.hpp"
#include "ManualTest/PeriphModuleTests/UdpReceiveModuleTest.hpp"
#include "ManualTest/CoreModuleTests/ConversionTest.hpp"
#include "ManualTest/CoreModuleTests/DataProcessorModuleTest.hpp"



class TestRunner {
public:
    TestRunner() {
        testsMap["periodic-thread"] = new PeriodicThreadTest();
        testsMap["pubsub"] = new PubSubTest();
        testsMap["tcp-receive"] = new TcpReceiveModuleTest();
        testsMap["udp-receive"] = new UdpReceiveModuleTest();
        testsMap["conversion"] = new ConversionTest();
        testsMap["data-processor"] = new DataProcessorModuleTest();
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
};


