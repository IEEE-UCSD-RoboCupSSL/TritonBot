#pragma once
#include "Misc/PubSubSystem/ThreadPool.hpp"
#include "Misc/PubSubSystem/Module.hpp"
#include <armadillo>



std::ostream& operator<<(std::ostream& os, const arma::vec& v);
std::ostream& operator<<(std::ostream& os, const arma::vec2& v);
std::ostream& operator<<(std::ostream& os, const arma::vec3& v);

class ManualTest {
public:
    ITPS::FieldPublisher<bool> *ballcapStatusPubMock;
    
    virtual bool test(ThreadPool& threadPool) = 0;
    virtual ~ManualTest() {}
};
