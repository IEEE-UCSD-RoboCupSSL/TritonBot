#pragma once

#include "ControlModule/pid_system.hpp"
#include "ProtoGenerated/vFirmware_API.pb.h"


class Virtual_PID_System : public PID_System {
    public: 
        Virtual_PID_System();

        void init_subscribers(void);
        
        MotionEKF::MotionData get_sensor_feedbacks(void);

        

    private: 
        
};