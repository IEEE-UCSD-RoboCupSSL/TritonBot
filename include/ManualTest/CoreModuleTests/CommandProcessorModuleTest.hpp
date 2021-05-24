#pragma once
#include "ManualTest/ManualTest.hpp"
#include "Config/Config.hpp"

class CommandProcessorModuleTest : public ManualTest {
public:
    CommandProcessorModuleTest(Config cfg) : config(cfg) {}
    bool test(ThreadPool& threadPool);
protected:
    Config config;
};
