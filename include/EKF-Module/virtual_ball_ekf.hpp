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
        boost::shared_ptr<boost::asio::ip::udp::socket> socket; 
        boost::shared_ptr<boost::asio::ip::udp::endpoint> grsim_endpoint;
        boost::shared_ptr<boost::asio::deadline_timer> timer;
        boost::asio::io_service io_service;
        
        arma::vec prev_disp = {0, 0};
        BallEKF::BallData ball_data;
        B_Log logger;
        void loop();
        void velocity_calc_timer_task();

};