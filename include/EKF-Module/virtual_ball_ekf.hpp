#pragma once
#include "EKF-Module/ball_ekf_module.hpp"
#include <armadillo>
#include "Utility/boost_logger.hpp"

// Pseudo EKF
class VirtualBallEKF : public BallEKF_Module {
    public: 
        VirtualBallEKF(); 
        ~VirtualBallEKF();

        void task() {}
        void task(ThreadPool& thread_pool);

    private:

        BallEKF::BallData ball_data;
        B_Log logger;


};