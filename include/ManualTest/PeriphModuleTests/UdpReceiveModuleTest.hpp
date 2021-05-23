#pragma once
#include "ManualTest/ManualTest.hpp"
#include "Config/Config.hpp"

class UdpReceiveModuleTest : public ManualTest {
public:
    UdpReceiveModuleTest(Config cfg) : config(cfg) {}
    bool test(ThreadPool& threadPool);
protected:
    Config config;
};