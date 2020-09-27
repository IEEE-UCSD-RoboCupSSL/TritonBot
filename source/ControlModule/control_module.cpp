#include "ControlModule/control_module.hpp"
#include "Config/config.hpp"

ControlModule::ControlModule(void) : enable_signal_sub("safety", "enable", 1), // MQ Mode
                                     sensor_sub("virtual-motion ekf", "motion prediction"), // Trivial Mode
                                     dribbler_signal_sub("CTRL", "dribbler"), // Trivial Mode
                                     kicker_setpoint_sub("CTRL", "kicker"), // Trivial Mode
                                     trans_setpoint_sub("CTRL", "trans"), // Trivial Mode
                                     rotat_setpoint_sub("CTRL", "rotat"), // Trivial Mode
                                     output_pub("vfirm-client", "commands")
{
    while(!enable_signal_sub.subscribe());
    while(!dribbler_signal_sub.subscribe());
    while(!kicker_setpoint_sub.subscribe());
    while(!trans_setpoint_sub.subscribe());
    while(!rotat_setpoint_sub.subscribe());
    while(!sensor_sub.subscribe());
}


bool ControlModule::get_enable_signal(void) {
    return enable_signal_sub.pop_msg(SAFETY_EN_TIMEOUT, false); // returns false if timeout
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

MotionEKF::MotionData ControlModule::get_sensor_feedbacks(void) {
    return sensor_sub.latest_msg();
}

void ControlModule::publish_output(VF_Commands& cmd) {
    output_pub.publish(cmd);
}

