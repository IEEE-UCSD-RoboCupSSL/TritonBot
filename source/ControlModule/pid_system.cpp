#include "ControlModule/pid_system.hpp"
#include "PubSubModule/thread_pool.hpp"
#include "ProtoGenerated/vFirmware_API.pb.h"
#include "Utility/systime.hpp"
#include "Utility/boost_logger.hpp"
#include "Config/config.hpp"
#include "ControlModule/pid.hpp"

#include <armadillo>

PID_System::PID_System() : ControlModule() {}



void PID_System::task(ThreadPool& thread_pool) {
    B_Log logger;
    logger.add_tag("PID_System Module");
    logger(Info) << "\033[0;32m Thread Started \033[0m";

    init_subscribers();


    VF_Commands halt_cmd;
    Vec_2D zero_vec;
    std::string write;
    zero_vec.set_x(0.00);
    zero_vec.set_y(0.00);
    halt_cmd.set_init(true);
    halt_cmd.set_allocated_translational_output(&zero_vec);
    halt_cmd.set_rotational_output(0.00);
    halt_cmd.set_allocated_kicker(&zero_vec);
    halt_cmd.set_dribbler(false);


    MotionEKF::MotionData feedback;
    arma::vec kicker_setpoint;
    bool dribbler_set_on;
    CTRL::SetPoint<arma::vec> trans_setpoint;
    CTRL::SetPoint<float> rotat_setpoint;
    Vec_2D kicker_out;


    VF_Commands output_cmd;
    output_cmd = halt_cmd; // default to halt

    PID_Controller<float> rotat_disp_pid(PID_RD_KP, PID_RD_KI, PID_RD_KD);
    PID_Controller<float> rotat_vel_pid(PID_RV_KP, PID_RV_KI, PID_RV_KD);
    PID_Controller<arma::vec> trans_disp_pid(PID_TD_KP, PID_TD_KI, PID_TD_KD);
    PID_Controller<arma::vec> trans_vel_pid(PID_TV_KP, PID_TV_KI, PID_TV_KD);

    float rotat_disp_out, rotat_vel_out;
    arma::vec trans_disp_out, trans_vel_out;
    Vec_2D trans_disp_proto_out, trans_vel_proto_out;

    delay(500); // wait a bit for other concurrent systems to get ready

    while(1) {

        rotat_disp_pid.init(CTRL_FREQUENCY);
        rotat_vel_pid.init(CTRL_FREQUENCY);
        trans_disp_pid.init(CTRL_FREQUENCY);
        trans_vel_pid.init(CTRL_FREQUENCY);

        while(get_enable_signal()) {
            feedback = get_sensor_feedbacks();

            kicker_setpoint = get_kicker_setpoint();
            dribbler_set_on = get_dribbler_signal();
            trans_setpoint = get_trans_setpoint();
            rotat_setpoint = get_rotat_setpoint();

            kicker_out.set_x(kicker_setpoint(0));
            kicker_out.set_y(kicker_setpoint(1));

            output_cmd.set_allocated_kicker(&kicker_out);
            output_cmd.set_dribbler(dribbler_set_on);

            // PID calculations 
            rotat_disp_out = rotat_disp_pid.calculate(rotat_setpoint.value - feedback.rotat_disp,
                                                     feedback.rotat_vel);
            rotat_vel_out = rotat_disp_pid.calculate(rotat_setpoint.value - feedback.rotat_vel);

            trans_disp_out = trans_disp_pid.calculate(feedback.trans_disp, feedback.trans_vel);
            trans_vel_out = trans_vel_pid.calculate(feedback.trans_vel);


            /* PID output selections
             *
             * velocity and displacement are not linear independent, 
             * so generally they should be mutually exclusive.
             * 
             * translational and rotational variables are linear independent,
             * so they can have controller running at the same time
             */ 
           
            // translational velocity controller
            if(trans_setpoint.type == velocity) {
                trans_vel_proto_out.set_x((float)trans_vel_out(0));
                trans_vel_proto_out.set_y((float)trans_vel_out(1));
                output_cmd.set_allocated_translational_output(&trans_vel_proto_out);
            }

            // translational displacement controller
            if(trans_setpoint.type == displacement) {
                trans_disp_proto_out.set_x((float)trans_disp_out(0));
                trans_disp_proto_out.set_y((float)trans_disp_out(1));
                output_cmd.set_allocated_translational_output(&trans_disp_proto_out);
            }

            // rotational velocity controller
            if(rotat_setpoint.type == velocity) {
                output_cmd.set_rotational_output(rotat_vel_out);
            } 

            // rotational displacement controller
            if(rotat_setpoint.type == displacement) {
                output_cmd.set_rotational_output(rotat_disp_out);
            }

            publish_output(output_cmd);
            delay(1.00/CTRL_FREQUENCY * 1000.00);


            output_cmd.release_translational_output();
            output_cmd.release_kicker();
        }
        
        publish_output(halt_cmd);
    }

    halt_cmd.release_translational_output();
    halt_cmd.release_kicker();
}