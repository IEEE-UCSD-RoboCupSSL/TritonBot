#pragma once
#include "ManualTest/ManualTest.hpp"

class ConversionTest : public ManualTest {
public:
    bool test(ThreadPool& threadPool);
};