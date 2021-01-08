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
        ITPS::NonBlockingSubscriber<arma::vec> ball_pos_sub;
        ITPS::NonBlockingSubscriber<arma::vec> ball_velo_sub;
        ITPS::NonBlockingSubscriber<MotionEKF::MotionData> bot_data_sub;
        ITPS::NonBlockingPublisher< Motion::MotionCMD > command_pub;
        ITPS::NonBlockingPublisher<bool> dribbler_signal_pub;
        ITPS::NonBlockingPublisher<bool> status_signal_pub;
        B_Log logger;

        /*
         *  Author: Haoen(Samuel) Luo
         *  Function to check if the ball is dribbled by the dribbler using simple mathematical heuristics. Virtual version.
         *  @param ball_pos: The current position of the ball in robot body frame.
         *  @param latest_motion_data: Robot motion data obtained from the Virtual EKF module. Currently only the robot's
         *  position in its body frame is being utilized.
         *  @return: A boolean indicating whether the ball is determined to be dribbled.
         *
         *  @Note to developer: The dribbler is about 80 units wide and about 100 units away from the center of the robot.
         */
        bool check_ball_captured_V(arma::vec ball_pos, MotionEKF_Module::MotionData latest_motion_data);

        double calc_angle(double delta_y, double delta_x);
        
};

using BallCapture = BallCaptureModule;