#pragma once
#include "PubSubModule/module.hpp"
#include <armadillo>

// Pseudo EKF
class VirtualMotionEFK : public Module {
    public: 
        void task() {}
        void task(ThreadPool& thread_pool);

        struct MotionData {
            arma::vec trans_disp;
            arma::vec trans_vel;
            float rotat_disp;
            float rotat_vel;
        };
};