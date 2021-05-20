#pragma once
#include "ManualTest/ManualTest.hpp"

class PubSubTest : public ManualTest {
public:
    bool test(ThreadPool& threadPool);
};