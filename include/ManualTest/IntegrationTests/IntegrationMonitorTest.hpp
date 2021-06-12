#pragma once
#include "ManualTest/ManualTest.hpp"
#include "Config/Config.hpp"

class IntegrationMonitorTest : public ManualTest {
public:
    IntegrationMonitorTest(Config& cfg) : config(cfg) {}
    bool test(ThreadPool& threadPool);
protected:
    Config config;
};