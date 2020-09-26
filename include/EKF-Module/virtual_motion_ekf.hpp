#pragma once
#include "PubSubModule/module.hpp"
#include "EKF-Module/motion_ekf.hpp"
#include <armadillo>

// Pseudo EKF
class VirtualMotionEFK : public MotionEFK {
    public: 
        void task() {}
        void task(ThreadPool& thread_pool);

};