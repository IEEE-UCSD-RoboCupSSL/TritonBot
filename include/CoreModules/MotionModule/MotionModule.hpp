#pragma once

#include "Misc/PubSubSystem/Module.hpp"
#include <armadillo>
#include "ProtoGenerated/vFirmware_API.pb.h"
#include "CoreModules/ControlModule/ControlModule.hpp"
#include "CoreModules/DataCmdTypes.hpp"




class MotionModule : public Module {
    public:
        MotionModule();
        virtual ~MotionModule();

        virtual void task() {}
        virtual void task(ThreadPool& threadPool);

    protected:
        virtual void init_subscribers(void);

        virtual void move(arma::vec setpoint_3d, CtrlMode mode, ReferenceFrame setpoint_ref_frame = WorldFrame); // default: setpoint frame is world frame



    private:
        arma::vec bodyframe_origin_w = {0, 0, 0}; // this is a world frame coordinate(w/ angle), which is used to calculate info about body frame
        SetPoint<float> rotat_setpoint;
        SetPoint<arma::vec> trans_setpoint;
        ITPS::FieldSubscriber< BotData > sensor_sub;
        ITPS::FieldSubscriber< arma::vec > robot_origin_w_sub; // robot's origin point (pos(0,0)) with respect to the worldframe (i.e. camera frame)
        ITPS::FieldSubscriber< MotionCommand > command_sub;
        ITPS::FieldPublisher<SetPoint<arma::vec>> trans_setpoint_pub;
        ITPS::FieldPublisher<SetPoint<float>> rotat_setpoint_pub;
        ITPS::FieldPublisher<bool> no_slowdown_pub; // work-around for no slowdown modes
};

using Motion = MotionModule;



