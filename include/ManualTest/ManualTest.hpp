#pragma once
#include "Misc/PubSubSystem/ThreadPool.hpp"
#include "Misc/PubSubSystem/Module.hpp"
#include <armadillo>
#include "Misc/Utility/Common.hpp"



class ManualTest {
public:
    ITPS::FieldPublisher<bool> *ballCapStatusPubMock;
    
    virtual bool test(ThreadPool& threadPool) = 0;
    virtual ~ManualTest() {}
};
