#pragma once
#include "EKF-Module/motion_ekf_module.hpp"
#include <armadillo>

// Pseudo EKF
class VirtualMotionEKF : public MotionEKF_Module {
    public: 
        VirtualMotionEKF(); 
        ~VirtualMotionEKF();

        void task() override {}

    [[noreturn]] void task(ThreadPool& thread_pool) override;

    private:
        boost::shared_ptr<boost::asio::ip::udp::socket> socket;
        boost::shared_ptr<boost::asio::ip::udp::endpoint> grsim_endpoint;
        boost::shared_ptr<boost::asio::deadline_timer> timer;
        boost::asio::io_service io_service;

        arma::vec prev_disp = {0, 0};
        MotionEKF::MotionData motion_data;
        void getPositionData();
        void loop();
        void velocity_calc_timer_task();

};