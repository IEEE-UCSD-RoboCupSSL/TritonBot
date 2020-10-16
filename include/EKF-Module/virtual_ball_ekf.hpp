#pragma once
#include "EKF-Module/ball_ekf_module.hpp"
#include <armadillo>

// Pseudo EKF
class VirtualBallEKF : public BallEKF_Module {
    public: 
        VirtualBallEKF(); 
        ~VirtualBallEKF();

        void task() {}
        void task(ThreadPool& thread_pool);

};