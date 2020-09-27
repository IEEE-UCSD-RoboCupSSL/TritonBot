#include "ControlModule/pid_system.hpp"
#include "PubSubModule/thread_pool.hpp"
#include "ProtoGenerated/vFirmware_API.pb.h"
#include "Utility/systime.hpp"
#include "Utility/boost_logger.hpp"
#include "Config/config.hpp"

PID_System::PID_System() : ControlModule() {}



void PID_System::task(ThreadPool& thread_pool) {
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

    while(1) {
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

            // translational velocity controller
            if(trans_setpoint.type == velocity) {

            }

            // translational displacement controller
            if(trans_setpoint.type == displacement) {

            }

            // rotational velocity controller
            if(rotat_setpoint.type == velocity) {

            }

            // rotational displacement controller
            if(rotat_setpoint.type == displacement) {

            }

            publish_output(output_cmd);
            delay(1.00/CTRL_FREQUENCY * 1000.00);
        }
        publish_output(halt_cmd);
    }

    output_cmd.release_translational_output();
    output_cmd.release_kicker();
    halt_cmd.release_translational_output();
    halt_cmd.release_kicker();
}