#include "ControlModule/control_module.hpp"
#include "Config/config.hpp"

ControlModule::ControlModule(void) : enable_signal_sub("Safety", "Enable", 1), // MQ Mode
                                     sensor_sub("MotionEKF", "MotionData"), // Trivial Mode
                                     dribbler_signal_sub("AI CMD", "Dribbler"), // Trivial Mode
                                     kicker_setpoint_sub("AI CMD", "Kicker"), // Trivial Mode
                                     trans_setpoint_sub("AI CMD", "Trans"), // Trivial Mode
                                     rotat_setpoint_sub("AI CMD", "Rotat"), // Trivial Mode
                                     refresh_origin_cnt_sub("AI CMD", "RefreshDispOrigin"),
                                     output_pub("FirmClient", "Commands")
{}

void ControlModule::init_subscribers(void) {
    while(!enable_signal_sub.subscribe());
    while(!dribbler_signal_sub.subscribe());
    while(!kicker_setpoint_sub.subscribe());
    while(!trans_setpoint_sub.subscribe());
    while(!rotat_setpoint_sub.subscribe());
    while(!refresh_origin_cnt_sub.subscribe());
    while(!sensor_sub.subscribe());

    // set default latest values when nothing is received
    dribbler_signal_sub.set_default_latest_msg(false);
    kicker_setpoint_sub.set_default_latest_msg({0.00, 0.00});
    
    CTRL::SetPoint<arma::vec> df_trans_sp; 
    df_trans_sp.type = velocity;
    df_trans_sp.value = {0.00, 0.00};
    trans_setpoint_sub.set_default_latest_msg(df_trans_sp);

    CTRL::SetPoint<float> df_rotat_sp;
    df_rotat_sp.type = velocity;
    df_rotat_sp.value = 0.00;
    rotat_setpoint_sub.set_default_latest_msg(df_rotat_sp);

    refresh_origin_cnt_sub.set_default_latest_msg(0);
    disp_origin = {0, 0};

    MotionEKF::MotionData dfmd;
    dfmd.rotat_disp = 0.00;
    dfmd.rotat_vel = 0.00;
    dfmd.trans_disp = {0.00, 0.00};
    dfmd.trans_vel = {0.00, 0.00};
    sensor_sub.set_default_latest_msg(dfmd);

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

MotionEKF::MotionData ControlModule::get_ekf_feedbacks(void) {
    return sensor_sub.latest_msg();
}

arma::vec ControlModule::get_disp_origin(void) {
    int cnt = refresh_origin_cnt_sub.latest_msg();
    if(cnt > prev_origin_cnt) { // AI Java code should make the cnt circular so that it won't exceed MAX INT
        MotionEKF::MotionData data = get_ekf_feedbacks();
        disp_origin = data.trans_disp; // update disp origin with the current trans disp
    }
    return disp_origin;
}

void ControlModule::publish_output(VF_Commands& cmd) {
    /* .publish(...) is pass by copy, so it's safe to 
     * pass cmd's reference to this function and 
     * updating cmd in the caller stack */
    output_pub.publish(cmd); 
}

