#pragma once

#include "PubSubSystem/module.hpp"
#include <armadillo>
#include "ProtoGenerated/vFirmware_API.pb.h"
#include "ControlModule/control_module.hpp"

class MotionModule : public Module {
    public:
        // Controller controls one of the following physics variables 
        enum CTRL_Mode {TDRD = 0, //TDRD: translational displacement & rotational displacement (at the same time)
                        TDRV = 1, //TDRV: translational displacement & rotational velocity (at the same time)
                        TVRD = 2, //TVRD: translational velocity & rotational displacement (at the same time)
                        TVRV = 3}; //TVRV: translational velocity & rotational velocity (at the same time)  
        
        enum ReferenceFrame {WorldFrame = 0, BodyFrame = 1};

        struct MotionCMD {
            arma::vec setpoint_3d; // <x, y, \theta> where \theta is the orientation angle 
            CTRL_Mode mode;
            ReferenceFrame ref_frame;
        };


        MotionModule();
        virtual ~MotionModule();

        virtual void task() = 0;
        virtual void task(ThreadPool& thread_pool);

    protected:
        virtual void init_subscribers(void);

        virtual void move(arma::vec setpoint_3d, CTRL_Mode mode, ReferenceFrame setpoint_ref_frame = WorldFrame); // default: setpoint frame is world frame



    private:
        arma::vec bodyframe_origin_w = {0, 0, 0}; // this is a world frame coordinate(w/ angle), which is used to calculate info about body frame
        CTRL::SetPoint<float> rotat_setpoint;
        CTRL::SetPoint<arma::vec> trans_setpoint;
        ITPS::Subscriber< MotionEKF::MotionData > sensor_sub;
        ITPS::Subscriber< arma::vec > robot_origin_w_sub; // robot's origin point (disp(0,0)) with respect to the worldframe (i.e. camera frame)
        ITPS::Subscriber< MotionCMD > command_sub;
        ITPS::Publisher<CTRL::SetPoint<arma::vec>> trans_setpoint_pub;
        ITPS::Publisher<CTRL::SetPoint<float>> rotat_setpoint_pub;
        


};

using Motion = MotionModule;
