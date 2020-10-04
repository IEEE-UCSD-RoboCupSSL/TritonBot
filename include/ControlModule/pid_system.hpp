#pragma once

#include "ControlModule/control_module.hpp"


class PID_System : public ControlModule {
    public: 
        PID_System();

        virtual void task() {}
        virtual void task(ThreadPool& thread_pool);

        virtual void init_subscribers(void);
        struct PID_Constants {
            double RD_Kp, RD_Ki, RD_Kd;
            double RV_Kp, RV_Ki, RV_Kd;
            double TD_Kp, TD_Ki, TD_Kd;
            double TV_Kp, TV_Ki, TV_Kd;
            double DIR_Kp, DIR_Ki, DIR_kd;
        };

    private: 
        ITPS::Subscriber<PID_Constants> pid_consts_sub;

};