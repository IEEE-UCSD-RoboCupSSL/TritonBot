#include "MotionModule/motion_module.hpp"
#include "Config/config.hpp"
#include "Utility/common.hpp"
#include "Utility/systime.hpp"

MotionModule::MotionModule() : trans_setpoint_pub("AI CMD", "Trans"), 
                               rotat_setpoint_pub("AI CMD", "Rotat"),
                               sensor_sub("MotionEKF", "MotionData"), // Trivial Mode
                               robot_origin_w_sub("ConnectionInit", "RobotOrigin(WorldFrame)"), // Trivial Mode
                               command_sub("CMD Server", "MotionCMD", 1) // MQ Mode
{}

void MotionModule::init_subscribers(void) {
    while(!sensor_sub.subscribe());
    while(!robot_origin_w_sub.subscribe());
    while(!command_sub.subscribe());
    MotionEKF::MotionData dfmd;
    dfmd.rotat_disp = 0.00;
    dfmd.rotat_vel = 0.00;
    dfmd.trans_disp = {0.00, 0.00};
    dfmd.trans_vel = {0.00, 0.00};
    sensor_sub.set_default_latest_msg(dfmd);

    arma::vec zero_vec = {0, 0};
    robot_origin_w_sub.set_default_latest_msg(zero_vec);

    // command_sub: MQ Mode don't need to set default msg
}


void MotionModule::task(ThreadPool& thread_pool) {
    UNUSED(thread_pool);
    B_Log logger;
    logger.add_tag("PID_System Module");
    logger(Info) << "\033[0;32m Thread Started \033[0m";
    init_subscribers();
    logger(Info) << "\033[0;32m Initialized \033[0m";
    delay(INIT_DELAY);
    logger(Info) << "\033[0;32m Loop Started \033[0m";
    
    while(1) {
        auto cmd = command_sub.pop_msg();
        move(cmd.setpoint_3d, cmd.mode, cmd.ref_frame);        
    }
}




void MotionModule::move(arma::vec setpoint_3d, CTRL_Mode mode, ReferenceFrame setpoint_ref_frame = WorldFrame) { // default: setpoint frame is world frame
    switch(mode) {
        case TDRD: trans_setpoint.type = CTRL::displacement;
                   rotat_setpoint.type = CTRL::displacement;
                   break;
        case TDRV: trans_setpoint.type = CTRL::displacement;
                   rotat_setpoint.type = CTRL::velocity;
                   break;
        case TVRD: trans_setpoint.type = CTRL::velocity;
                   rotat_setpoint.type = CTRL::displacement;
                   break;
        case TVRV: trans_setpoint.type = CTRL::velocity;
                   rotat_setpoint.type = CTRL::velocity;
                   break;

        trans_setpoint.value = {setpoint_3d(0), setpoint_3d(1)};
        rotat_setpoint.value = setpoint_3d(2);

        if(setpoint_ref_frame == WorldFrame) {
            arma::vec bot_origin = robot_origin_w_sub.latest_msg();
            double bot_orien = sensor_sub.latest_msg().rotat_disp;

            /* a not-so-obvious simplification was done by 
             * using bot_origin as the bot curr location */
            arma::mat A = wtb_homo_transform(bot_origin, bot_orien); // world to body homogeneous transformation

            // setpoint with respect to world reference frame
            arma::vec setpoint_w = {trans_setpoint.value(0), trans_setpoint.value(1), 1}; // homogeneous point end with a 1 (vector end with a 0)

            arma::vec setpoint_b = A * setpoint_w; // apply transformation to get the same point represented in the body frame

            // update setpoint to the setpoint in robot's perspective
            trans_setpoint.value = {setpoint_b(0), setpoint_b(1)};


            /* for rotational, we simply unify their zero orientation to avoid needing transformations */
        }

        trans_setpoint_pub.publish(trans_setpoint);
        rotat_setpoint_pub.publish(rotat_setpoint);

    }
}

