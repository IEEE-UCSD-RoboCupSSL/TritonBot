#pragma once

#include "PubSubSystem/module.hpp"
#include <armadillo>
#include "ProtoGenerated/vFirmware_API.pb.h"
#include "EKF-Module/motion_ekf_module.hpp"


class ControlModule : public Module {
    public: 
        enum SetPointType {velocity, displacement};
        template <typename ValueType>
        struct SetPoint {
            ValueType value;
            SetPointType type;
        };

        ControlModule(void);
        virtual ~ControlModule() {}

        virtual void task() = 0;
        virtual void task(ThreadPool& thread_pool) = 0;


    protected:
        VF_Commands halt_cmd;
        virtual void init_subscribers(void);
        bool get_enable_signal(void);
        bool is_headless_mode(void);
        virtual MotionEKF::MotionData get_ekf_feedbacks(void);
        arma::vec get_kicker_setpoint(void);
        bool get_dribbler_signal(void);       
        SetPoint<arma::vec> get_trans_setpoint(void);
        SetPoint<float> get_rotat_setpoint(void);
        void publish_output(VF_Commands& cmd);
        arma::mat headless_transform(double robot_orient);

    private:
        ITPS::BlockingSubscriber<bool> enable_signal_sub;
        ITPS::NonBlockingSubscriber<bool> is_headless_sub;
        ITPS::NonBlockingSubscriber< MotionEKF::MotionData > sensor_sub;
        ITPS::NonBlockingSubscriber<bool> dribbler_signal_sub;
        ITPS::NonBlockingSubscriber< arma::vec > kicker_setpoint_sub;
        ITPS::NonBlockingSubscriber< SetPoint<arma::vec> > trans_setpoint_sub;
        ITPS::NonBlockingSubscriber< SetPoint<float> > rotat_setpoint_sub;
        ITPS::BlockingPublisher< VF_Commands > output_pub;     
        
};

using CTRL = ControlModule;