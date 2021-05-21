#pragma once
#include "Misc/PubSubSystem/ThreadPool.hpp"

class ManualTest {
public:
    virtual bool test(ThreadPool& threadPool) = 0;
    virtual ~ManualTest() {}
};

