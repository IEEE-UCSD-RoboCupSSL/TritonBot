#pragma once

#include "PubSubSystem/module.hpp"
#include <armadillo>
#include "ProtoGenerated/vFirmware_API.pb.h"
#include "ControlModule/control_module.hpp"
#include "RulesKeeper/field.hpp"

class MotionModule : public Module {
    public:
        /*
         * Controller controls one of the following physics variables
         * * TDRD: translational displacement & rotational displacement (at the same time)
         * * TDRV: translational displacement & rotational velocity (at the same time)
         * * TVRD: translational velocity & rotational displacement (at the same time)
         * * TVRV: translational velocity & rotational velocity (at the same time)  
         */
        enum CTRL_Mode {TDRD, TDRV, TVRD, TVRV};
        enum ReferenceFrame {GlobalFrame, BodyFrame};

        MotionModule(Field& field);

        virtual void task() = 0;
        virtual void task(ThreadPool& thread_pool);



    private:
        Field *field;


};

using Motion = MotionModule;
