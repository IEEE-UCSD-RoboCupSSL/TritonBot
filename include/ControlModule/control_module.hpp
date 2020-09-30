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


        virtual void task() = 0;
        virtual void task(ThreadPool& thread_pool) = 0;

        virtual void init_subscribers(void);

        ControlModule(void);
        virtual ~ControlModule() {}

        bool get_enable_signal(void);
        
        virtual MotionEKF::MotionData get_sensor_feedbacks(void);

        arma::vec get_kicker_setpoint(void);
        bool get_dribbler_signal(void);       
        SetPoint<arma::vec> get_trans_setpoint(void);
        SetPoint<float> get_rotat_setpoint(void);
        
        void publish_output(VF_Commands& cmd);

    private:
        ITPS::Subscriber<bool> enable_signal_sub;
        ITPS::Subscriber< MotionEKF::MotionData > sensor_sub;
        
        ITPS::Subscriber<bool> dribbler_signal_sub;
        ITPS::Subscriber< arma::vec > kicker_setpoint_sub;
        ITPS::Subscriber< SetPoint<arma::vec> > trans_setpoint_sub;
        ITPS::Subscriber< SetPoint<float> > rotat_setpoint_sub;

        ITPS::Publisher< VF_Commands > output_pub;     

};

using CTRL = ControlModule;