#pragma once
#include "ManualTest/ManualTest.hpp"
#include "Config/Config.hpp"
class DataProcessorModuleTest : public ManualTest {
public:
    DataProcessorModuleTest(Config cfg) : config(cfg) {}
    bool test(ThreadPool& threadPool);
protected:
    Config config;
};
