#pragma once
#include "PubSubModule/module.hpp"
#include <armadillo>

class MotionEFK : public Module {
    public: 
        virtual void task() {}
        virtual void task(ThreadPool& thread_pool) {}

        struct MotionData {
            arma::vec trans_disp;
            arma::vec trans_vel;
            float rotat_disp;
            float rotat_vel;
        };
};