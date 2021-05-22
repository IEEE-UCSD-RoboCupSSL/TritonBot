#pragma once
#include "ManualTest/ManualTest.hpp"

class DataProcessorModuleTest : public ManualTest {
public:
    bool test(ThreadPool& threadPool);
};