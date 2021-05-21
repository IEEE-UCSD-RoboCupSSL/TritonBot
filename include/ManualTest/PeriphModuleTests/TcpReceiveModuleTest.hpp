#pragma once
#include "ManualTest/ManualTest.hpp"

class TcpReceiveModuleTest : public ManualTest {
public:
    bool test(ThreadPool& threadPool);
};