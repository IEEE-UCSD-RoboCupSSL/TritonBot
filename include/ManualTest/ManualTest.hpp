#pragma once
#include "Misc/PubSubSystem/ThreadPool.hpp"
#include "Misc/PubSubSystem/Module.hpp"

class ManualTest {
public:
    ITPS::FieldPublisher<bool> *ballcapStatusPubMock;
    
    virtual bool test(ThreadPool& threadPool) = 0;
    virtual ~ManualTest() {}
};

