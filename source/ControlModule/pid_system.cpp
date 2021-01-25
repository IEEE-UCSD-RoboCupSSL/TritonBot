#include "ControlModule/pid_system.hpp"
#include "PubSubSystem/thread_pool.hpp"
#include "ProtoGenerated/vFirmware_API.pb.h"
#include "Utility/systime.hpp"
#include "Utility/boost_logger.hpp"
#include "Config/config.hpp"
#include "ControlModule/pid.hpp"
#include "EKF-Module/motion_ekf_module.hpp"
#include "Utility/common.hpp"
#include <armadillo>



PID_System::PID_System() : ControlModule(),
                           pid_consts_sub("PID", "Constants")
{}

void PID_System::init_subscribers(void) {
    ControlModule::init_subscribers();
    try { 
        pid_consts_sub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
    }
    catch(std::exception& e) {
        B_Log logger;
        logger.add_tag("[pid_system.cpp]");
        logger.log(Error, e.what());
        std::exit(0);
    }
}



void PID_System::task(ThreadPool& thread_pool) {
    B_Log logger;
    logger.add_tag("PID_System Module");
    logger(Info) << "\033[0;32m Thread Started \033[0m";

    init_subscribers();
    logger(Info) << "\033[0;32m Initialized \033[0m";
    
    MotionEKF::MotionData feedback;
    arma::vec kicker_setpoint;
    bool dribbler_set_on;
    CTRL::SetPoint<arma::vec> trans_setpoint;
    CTRL::SetPoint<float> rotat_setpoint;
    Vec_2D kicker_out;


    VF_Commands output_cmd;
    output_cmd = halt_cmd; // default to halt

    PID_Controller<float> rotat_disp_pid(PID_RD_KP, PID_RD_KI, PID_RD_KD);
    PID_Controller<arma::vec> trans_disp_pid(PID_TD_KP, PID_TD_KI, PID_TD_KD);

    float rotat_disp_out = 0.0, rotat_vel_out = 0.0;
    arma::vec trans_disp_out, trans_vel_out;
    arma::vec output_3d;
    Vec_2D trans_proto_out;
    PID_Constants pid_consts;
    float corr_angle = 0.0;
    float angle_err = 0.0;
    float pid_amplifier;

    delay(INIT_DELAY); // controller shall not start before the 
                       // garbage data are refreshed by other modules 
                       // after running for a bit 
    logger(Info) << "\033[0;32m Control Loop Started \033[0m";
    while(1) { // has delay (good for reducing high CPU usage)

        rotat_disp_pid.init(CTRL_FREQUENCY);
        trans_disp_pid.init(CTRL_FREQUENCY);

        while(get_enable_signal()) {
            pid_amplifier = 1.00;
            if(get_no_slowdown()) {
                pid_amplifier = NS_PID_AMP;
            }
            pid_consts = pid_consts_sub.latest_msg();
            rotat_disp_pid.update_pid_consts(pid_consts.RD_Kp, pid_consts.RD_Ki, pid_consts.RD_Kd);
            trans_disp_pid.update_pid_consts(pid_amplifier * pid_consts.TD_Kp, pid_consts.TD_Ki, pid_consts.TD_Kd);

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
            // Rotation Controller
            if(rotat_setpoint.type == displacement) {
                angle_err = rotat_setpoint.value - feedback.rotat_disp; // Expected Value - Actual Value
                if(std::signbit(rotat_setpoint.value) != std::signbit(feedback.rotat_disp)) { 
                    // having opposite sign means one is in the 0 ~ 180 region and the other is in 0 ~ -180
                    float alt_error = angle_err > 0 ? (angle_err - 360) : (360 + angle_err);
                    // find the direction with the shortest angle_err value
                    if(std::fabs(angle_err) > std::fabs(alt_error)) {
                        angle_err = alt_error;
                    }
                }
                rotat_disp_out = rotat_disp_pid.calculate(angle_err);
            }
            else { // type == velocity
                rotat_disp_pid.init(CTRL_FREQUENCY);
                // if(rotat_setpoint.value > PID_MAX_ROT_PERC) {
                //     rotat_setpoint.value = PID_MAX_ROT_PERC;
                // } else if(rotat_setpoint.value < -PID_MAX_ROT_PERC) {
                //     rotat_setpoint.value = -PID_MAX_ROT_PERC;
                // }
                rotat_vel_out = rotat_setpoint.value;

            }

            // Translation Movement Controller
            if(trans_setpoint.type == displacement) {

                trans_disp_out = trans_disp_pid.calculate(trans_setpoint.value - feedback.trans_disp );

                // correct deviation due to rotation momentum        
                if(rotat_setpoint.type == displacement) {
                    corr_angle = -rotat_disp_out * PID_TDRD_CORR;
                }
                else {
                    corr_angle = -rotat_vel_out * PID_TDRV_CORR;
                }
                trans_disp_out = rotation_matrix_2D(corr_angle) * trans_disp_out; // correct direction by rotation matrix 

            }
            else {
                // type == velocity
                trans_disp_pid.init(CTRL_FREQUENCY);
                trans_vel_out = trans_setpoint.value;

                // correct deviation due to rotation momentum        
                if(rotat_setpoint.type == displacement) {
                    corr_angle = -rotat_disp_out * PID_TVRD_CORR;
                }
                else {
                    corr_angle = -rotat_vel_out * PID_TVRV_CORR;
                }
                trans_vel_out = rotation_matrix_2D(corr_angle) * trans_vel_out; // correct direction by rotation matrix 

            }



            

            /* PID output selections
             *
             * velocity and displacement are not independent, 
             * so generally they should be mutually exclusive.
             * 
             * translational and rotational variables are linear independent,
             * so they can have different controllers running at the same time
             */ 
           
            // translational velocity 
            if(trans_setpoint.type == velocity) {
                output_3d = {trans_vel_out(0), trans_vel_out(1), 0}; // 0 is just a space holder, rotation is set few lines down below
            } 
            // translational displacement 
            else if(trans_setpoint.type == displacement) { 
                output_3d = {trans_disp_out(0), trans_disp_out(1), 0};
            }
            // rotational velocity 
            if(rotat_setpoint.type == velocity) {
                output_3d(2) = rotat_vel_out;
            } 
            // rotational displacement 
            else if(rotat_setpoint.type == displacement) { 
                output_3d(2) = rotat_disp_out;
            }

            


            /* Effect of Normalizing: more power spent on rotation results in less spent on translation, vice versa */
            if(arma::norm(output_3d) > 100.00) {
                // Normalize the output vector to limit the maximum output vector norm to 100.00
                output_3d = arma::normalise(output_3d);
                output_3d *= 100.00;
            }

            // convert to protobuffer-defined cmd type
            trans_proto_out.set_x(output_3d(0));
            trans_proto_out.set_y(output_3d(1));
            output_cmd.set_allocated_translational_output(&trans_proto_out);
            output_cmd.set_rotational_output(output_3d(2));

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