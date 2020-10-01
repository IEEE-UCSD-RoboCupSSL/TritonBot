#include "ControlModule/pid_system.hpp"
#include "PubSubSystem/thread_pool.hpp"
#include "ProtoGenerated/vFirmware_API.pb.h"
#include "Utility/systime.hpp"
#include "Utility/boost_logger.hpp"
#include "Config/config.hpp"
#include "ControlModule/pid.hpp"
#include "EKF-Module/motion_ekf_module.hpp"
#include <armadillo>


PID_System::PID_System() : ControlModule(),
                           pid_consts_sub("PID", "Constants")
{}

void PID_System::init_subscribers(void) {
    ControlModule::init_subscribers();
    while(!pid_consts_sub.subscribe());
    PID_Constants dfconst;
    dfconst.RD_Kd = PID_RD_KD;
    dfconst.RD_Ki = PID_RD_KI;
    dfconst.RD_Kp = PID_RD_KP; 
    dfconst.RV_Ki = PID_RV_KI;
    dfconst.RV_Kp = PID_RV_KP;
    dfconst.RV_Kd = PID_RV_KD;
    dfconst.TD_Kd = PID_TD_KD;
    dfconst.TD_Ki = PID_TD_KI;
    dfconst.TD_Kp = PID_TD_KP;
    dfconst.TV_Kd = PID_TV_KD;
    dfconst.TV_Ki = PID_TV_KI;
    dfconst.TV_Kp = PID_TV_KP;

    pid_consts_sub.set_default_latest_msg(dfconst);
}



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
    PID_Constants pid_consts;

    delay(500); // wait a bit for other concurrent systems to get ready

    while(1) {

        rotat_disp_pid.init(CTRL_FREQUENCY);
        rotat_vel_pid.init(CTRL_FREQUENCY);
        trans_disp_pid.init(CTRL_FREQUENCY);
        trans_vel_pid.init(CTRL_FREQUENCY);

        while(get_enable_signal()) {
            pid_consts = pid_consts_sub.latest_msg();
            rotat_disp_pid.update_pid_consts(pid_consts.RD_Kp, pid_consts.RD_Ki, pid_consts.RD_Kd);
            rotat_vel_pid.update_pid_consts(pid_consts.RV_Kp, pid_consts.RV_Ki, pid_consts.RV_Kd);
            trans_disp_pid.update_pid_consts(pid_consts.TD_Kp, pid_consts.TD_Ki, pid_consts.RV_Kd);
            trans_vel_pid.update_pid_consts(pid_consts.TV_Kp, pid_consts.TV_Ki, pid_consts.TV_Kd);

            feedback = get_ekf_feedbacks();

            kicker_setpoint = get_kicker_setpoint();
            dribbler_set_on = get_dribbler_signal();
            trans_setpoint = get_trans_setpoint();
            rotat_setpoint = get_rotat_setpoint();

            kicker_out.set_x(kicker_setpoint(0));
            kicker_out.set_y(kicker_setpoint(1));

            output_cmd.set_allocated_kicker(&kicker_out);
            output_cmd.set_dribbler(dribbler_set_on);

            // PID calculations : Error = SetPoint - CurrPoint = ExpectedValue - ActualValue
            if(rotat_setpoint.type == displacement) {
                rotat_disp_out = rotat_disp_pid.calculate(rotat_setpoint.value - feedback.rotat_disp);
                rotat_vel_pid.init(CTRL_FREQUENCY); // re-init the conjugate type, 
                                                    // init method reset the integral sum and derivative delta to 0, 
                                                    //  if these values are from long ago, best to refresh them for a new start
            }
            else {
                rotat_disp_pid.init(CTRL_FREQUENCY);
                rotat_vel_out = rotat_vel_pid.calculate(rotat_setpoint.value - feedback.rotat_vel);
            }

            if(trans_setpoint.type == displacement) {
                trans_disp_out = trans_disp_pid.calculate(trans_setpoint.value - feedback.trans_disp);
                trans_vel_pid.init(CTRL_FREQUENCY);
            }
            else {
                trans_disp_pid.init(CTRL_FREQUENCY);
                trans_vel_out = trans_vel_pid.calculate(trans_setpoint.value - feedback.trans_vel);
            }

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