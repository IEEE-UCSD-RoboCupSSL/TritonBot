#pragma once
#include "PubSubSystem/module.hpp"
#include <armadillo>



class MotionEKF_Module : public Module {
    public: 
        virtual void task() = 0;
        virtual void task(ThreadPool& thread_pool) = 0;

        virtual ~MotionEKF_Module() {}

        struct MotionData {
            arma::vec trans_disp;
            arma::vec trans_vel;
            float rotat_disp;
            float rotat_vel;
        };

};

// alias
using MotionEKF = MotionEKF_Module;