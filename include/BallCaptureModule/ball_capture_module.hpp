#pragma once

#include "PubSubSystem/module.hpp"
#include <armadillo>
#include "MotionModule/motion_module.hpp"
#include "EKF-Module/ball_ekf_module.hpp"
#include "EKF-Module/motion_ekf_module.hpp"

class BallCaptureModule : public Module {

    BallCaptureModule();
    virtual ~BallCaptureModule();

    virtual void task(ThreadPool& thread_pool);


    protected:
        virtual void init_subscribers();
        bool is_this_module_enabled();

    private:
        ITPS::NonBlockingSubscriber<bool> enable_sub;
        ITPS::NonBlockingPublisher< Motion::MotionCMD > command_pub;
        
};

using BallCapture = BallCaptureModule;