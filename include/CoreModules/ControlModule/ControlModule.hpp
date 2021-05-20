#pragma once

#include "Misc/PubSubSystem/Module.hpp"
#include <armadillo>
#include "ProtoGenerated/vFirmware_API.pb.h"
#include "CoreModules/EKF-Module/MotionEkfModule.hpp"
#include "ControlModule.hpp"



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

        
        virtual void task(ThreadPool& threadPool) = 0;


    protected:
        VF_Commands halt_cmd;
        virtual void init_subscribers(void);
        bool get_enable_signal(void);
        virtual MotionEKF::MotionData get_ekf_feedbacks(void);
        arma::vec get_kicker_setpoint(void);
        bool get_dribbler_signal(void);       
        SetPoint<arma::vec> get_trans_setpoint(void);
        SetPoint<float> get_rotat_setpoint(void);
        void publish_output(VF_Commands& cmd);
        arma::mat headless_transform(double robot_orient);
        bool get_no_slowdown(void);

    private:
        ITPS::NonBlockingSubscriber<bool> enable_signal_sub;
        ITPS::NonBlockingSubscriber< MotionEKF::MotionData > sensor_sub;
        ITPS::NonBlockingSubscriber<bool> dribbler_signal_sub;
        ITPS::NonBlockingSubscriber< arma::vec > kicker_setpoint_sub;
        ITPS::NonBlockingSubscriber< SetPoint<arma::vec> > trans_setpoint_sub;
        ITPS::NonBlockingSubscriber< SetPoint<float> > rotat_setpoint_sub;
        ITPS::BlockingPublisher< VF_Commands > output_pub; 
        ITPS::NonBlockingSubscriber<bool> no_slowdown_sub;     
        
};

using CTRL = ControlModule;

class PID_System : public ControlModule {
public:
    PID_System();

    virtual void task() {}
    virtual void task(ThreadPool& threadPool);

    virtual void init_subscribers(void);
    struct PID_Constants {
        double RD_Kp, RD_Ki, RD_Kd;
        // double RV_Kp, RV_Ki, RV_Kd;
        double TD_Kp, TD_Ki, TD_Kd;
        // double TV_Kp, TV_Ki, TV_Kd;
        // double DIR_Kp, DIR_Ki, DIR_Kd;
    };

private:
    ITPS::NonBlockingSubscriber<PID_Constants> pid_consts_sub;

};