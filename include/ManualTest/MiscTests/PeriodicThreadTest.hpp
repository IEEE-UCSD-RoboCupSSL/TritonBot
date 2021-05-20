#pragma once
#include "ManualTest/ManualTest.hpp"

class PeriodicThreadTest : public ManualTest {
public:
    bool test(ThreadPool& threadPool);
};