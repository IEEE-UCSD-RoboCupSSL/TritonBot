#pragma once
#include "ManualTest/ManualTest.hpp"
#include "Config/Config.hpp"
class VFTest : public ManualTest {
public:
    VFTest(Config cfg) : config(cfg) {}
    bool test(ThreadPool& threadPool);
protected:
    Config config;
};