#pragma once

#include "PubSubModule/module.hpp"
#include <armadillo>

class ControlModule : public Module {
    public: 
        void task() {}
        void task(ThreadPool& thread_pool);

        
};