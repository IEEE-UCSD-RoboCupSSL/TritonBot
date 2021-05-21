#pragma once
#include "ManualTest/ManualTest.hpp"

class UdpReceiveModuleTest : public ManualTest {
public:
    bool test(ThreadPool& threadPool);
};