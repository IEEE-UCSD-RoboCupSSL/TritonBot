#pragma once
#include "ManualTest/ManualTest.hpp"
#include "Config/Config.hpp"

class MotionControllerModuleTest : public ManualTest {
public:
    MotionControllerModuleTest(Config cfg) : config(cfg) {}
    bool test(ThreadPool& threadPool);
protected:
    Config config;
};
