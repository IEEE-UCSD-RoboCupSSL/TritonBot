#pragma once
#include "ManualTest/ManualTest.hpp"
#include "Config/Config.hpp"
class TcpReceiveModuleTest : public ManualTest {
public:
    TcpReceiveModuleTest(Config& cfg) : config(cfg) {}
    bool test(ThreadPool& threadPool);
protected:
    Config config;
};