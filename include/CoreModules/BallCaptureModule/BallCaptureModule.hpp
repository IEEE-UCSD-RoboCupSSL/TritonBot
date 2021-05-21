#pragma once

#include "Misc/PubSubSystem/Module.hpp"
#include <armadillo>
#include "CoreModules/MotionModule/MotionModule.hpp"
#include "CoreModules/EKF-Module/BallEkfModule.hpp"
#include "CoreModules/EKF-Module/MotionEkfModule.hpp"
#include "Misc/Utility/Common.hpp"
#include <cmath>

class BallCaptureModule : public Module {

    public:
        BallCaptureModule();
        virtual ~BallCaptureModule();

    [[noreturn]] void task(ThreadPool& threadPool) override;


    protected:
        virtual void init_subscribers();

    private:
        ITPS::FieldSubscriber<bool> enable_sub;
        ITPS::FieldSubscriber<BallEKF::BallData> ball_data_sub;
        ITPS::FieldSubscriber<MotionEKF::BotData> bot_data_sub;
        ITPS::FieldPublisher< Motion::MotionCMD > command_pub;
        ITPS::FieldPublisher<bool> ballcap_status_pub;

        ITPS::FieldPublisher<bool> drib_enable_pub;
        BLogger logger;

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
        bool check_ball_captured_V(arma::vec ball_pos, MotionEKF_Module::BotData latest_motion_data);

        bool check_close_enough(arma::vec ball_pos, MotionEKF_Module::BotData latest_motion_data);

        double calc_angle(double delta_y, double delta_x);
        
};

using BallCapture = BallCaptureModule;