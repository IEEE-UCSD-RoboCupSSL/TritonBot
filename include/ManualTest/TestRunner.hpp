#pragma once

#include <iostream>
#include <unordered_map>
#include <string>
#include "Misc/PubSubSystem/ThreadPool.hpp"
#include "ManualTest/ManualTest.hpp"
#include "ManualTest/MiscTests/PeriodicThreadTest.hpp"
#include "ManualTest/MiscTests/PubSubTest.hpp"



class TestRunner {
public:
    TestRunner() {
        tests_map["periodic-thread"] = new PeriodicThreadTest();
        tests_map["pubsub"] = new PubSubTest();
    }
    ~TestRunner() {
        for(auto it = tests_map.begin(); it != tests_map.end(); it++) {
            delete it->second;
        }
    }
    void run(ThreadPool& threadPool);

protected:
    ManualTest* findTest(std::string testName);
    std::unordered_map<std::string, ManualTest*> tests_map;  

private:
    void printAllAvailableTests();
};

