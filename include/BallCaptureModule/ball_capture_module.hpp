#pragma once

#include "PubSubSystem/module.hpp"
#include <armadillo>
#include "MotionModule/motion_module.hpp"
#include "EKF-Module/ball_ekf_module.hpp"
#include "EKF-Module/motion_ekf_module.hpp"
#include "Utility/common.hpp"
#include <cmath>

class BallCaptureModule : public Module {

    public:
        BallCaptureModule();
        virtual ~BallCaptureModule();

    [[noreturn]] void task(ThreadPool& thread_pool) override;


    protected:
        virtual void init_subscribers();
        bool is_this_module_enabled();

    private:
        ITPS::NonBlockingSubscriber<bool> enable_sub;
        ITPS::NonBlockingSubscriber<BallEKF_Module::BallData> ball_data_sub;
        ITPS::NonBlockingSubscriber<MotionEKF_Module::MotionData> motion_data_sub;
        ITPS::NonBlockingPublisher< Motion::MotionCMD > command_pub;
        ITPS::NonBlockingPublisher<bool> dribbler_signal_pub;
        ITPS::NonBlockingPublisher<bool> status_signal_pub;
        B_Log logger;

        bool check_ball_captured(BallEKF_Module::BallData latest_ball_data, MotionEKF_Module::MotionData latest_motion_data);
        
};

using BallCapture = BallCaptureModule;