#include "ControlModule/control_module.hpp"
#include "Config/config.hpp"
#include "Utility/boost_logger.hpp"
#include "Utility/common.hpp"

ControlModule::ControlModule(void) : enable_signal_sub("AI Connection", "SafetyEnable"), 
                                     sensor_sub("MotionEKF", "MotionData"), 
                                     dribbler_signal_sub("BallCapture", "Dribbler"), 
                                     kicker_setpoint_sub("Kicker", "KickingSetPoint"), 
                                     trans_setpoint_sub("AI CMD", "Trans"), 
                                     rotat_setpoint_sub("AI CMD", "Rotat"), 
                                     output_pub("FirmClient", "Commands")
{
    
    Vec_2D zero_vec;
    zero_vec.set_x(0.00);
    zero_vec.set_y(0.00);
    halt_cmd.set_init(true);
    halt_cmd.set_allocated_translational_output(&zero_vec);
    halt_cmd.set_rotational_output(0.00);
    halt_cmd.set_allocated_kicker(&zero_vec);
    halt_cmd.set_dribbler(false);

    halt_cmd.release_kicker();
    halt_cmd.release_translational_output();
}

void ControlModule::init_subscribers(void) {
    try {
        enable_signal_sub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        dribbler_signal_sub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        kicker_setpoint_sub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        trans_setpoint_sub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        rotat_setpoint_sub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        sensor_sub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
    }
    catch(std::exception& e) {
        B_Log logger;
        logger.add_tag("[control_module.cpp]");
        logger.log(Error, e.what());
        while(1);
    }
}

bool ControlModule::get_enable_signal(void) {
    return enable_signal_sub.latest_msg();
}

arma::vec ControlModule::get_kicker_setpoint(void) {
    return kicker_setpoint_sub.latest_msg();
}

bool ControlModule::get_dribbler_signal(void) {
    return dribbler_signal_sub.latest_msg();
}   

CTRL::SetPoint<arma::vec> ControlModule::get_trans_setpoint(void) {
    return trans_setpoint_sub.latest_msg();
}

CTRL::SetPoint<float> ControlModule::get_rotat_setpoint(void) {
    return rotat_setpoint_sub.latest_msg();
}

MotionEKF::MotionData ControlModule::get_ekf_feedbacks(void) {
    return sensor_sub.latest_msg();
}

// From WorldFrame(absolute zero degree direction) to BodyFrame(direction the kicker/dribbler points at) coordinates
arma::mat ControlModule::headless_transform(double robot_orient) {
    arma::mat rot = rotation_matrix_2D(robot_orient);
    arma::vec unit_vec_x = {1, 0};
    arma::vec unit_vec_y = {0, 1};
    arma::vec Uxy = rot * unit_vec_x;
    arma::vec Vxy = rot * unit_vec_y;

    arma::mat A_inv =  {{Uxy(0), Vxy(0)},
                        {Uxy(1), Vxy(1)}};

    return inv(A_inv);
}


void ControlModule::publish_output(VF_Commands& cmd) {
    /* .publish(...) is pass by copy, so it's safe to 
     * pass cmd's reference to this function and 
     * updating cmd in the caller stack */
    output_pub.publish(cmd); 
}

