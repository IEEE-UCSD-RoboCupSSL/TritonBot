#pragma once
#include "ManualTest/ManualTest.hpp"
#include "Config/Config.hpp"
class McuClientModuleTest : public ManualTest {
public:
    McuClientModuleTest(Config& cfg) : config(cfg) {}
    bool test(ThreadPool& threadPool);
protected:
    Config config;
};