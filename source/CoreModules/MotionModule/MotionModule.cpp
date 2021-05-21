#include "CoreModules/MotionModule/MotionModule.hpp"
#include "Config/Config.hpp"
#include "Misc/Utility/Common.hpp"
#include "Misc/Utility/Systime.hpp"
#include "Misc/Utility/BoostLogger.hpp"




static CTRL::SetPoint<arma::vec> default_trans_sp() {
    CTRL::SetPoint<arma::vec> rtn;
    arma::vec zero_vec = {0, 0};
    rtn.value = zero_vec;
    rtn.type = CTRL::SetPointType::velocity;
    return rtn;
}

static CTRL::SetPoint<float> default_rot_sp() {
    CTRL::SetPoint<float> rtn;
    rtn.value = 0;
    rtn.type = CTRL::SetPointType::velocity;
    return rtn;
}

MotionModule::MotionModule() : trans_setpoint_pub("AI CMD", "Trans", default_trans_sp()), 
                               rotat_setpoint_pub("AI CMD", "Rotat", default_rot_sp()),
                               sensor_sub("MotionEKF", "MotionData"), // NonBlocking Mode
                               robot_origin_w_sub("From:TcpReceiveModule", "RobotOrigin(WorldFrame)"), // NonBlocking Mode
                               command_sub("CMD Server", "MotionCMD"), // NonBlocking Mode because this module needs to keep the loop running 
                                                                       // non-blocking to calculate transformation matrix that changes along
                                                                       // the orientation of a moving robot
                               no_slowdown_pub("AI CMD", "NoSlowdown", false)
{}

MotionModule::~MotionModule() {}

void MotionModule::init_subscribers(void) {
    try {
        sensor_sub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        robot_origin_w_sub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        command_sub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
    }
    catch(std::exception& e) {
        BLogger logger;
        logger.addTag("[motion_module.cpp]");
        logger.log(Error, e.what());
        std::exit(0);
    }

}


void MotionModule::task(ThreadPool& threadPool) {
    UNUSED(threadPool);
    BLogger logger;
    logger.addTag("Motion Module");
    logger(Info) << "\033[0;32m Thread Started \033[0m";
    init_subscribers();
    logger(Info) << "\033[0;32m Initialized \033[0m";
    delay(INIT_DELAY);
    logger(Info) << "\033[0;32m Loop Started \033[0m";
    
    while(1) { // has delay (good for reducing high CPU usage)
        auto cmd = command_sub.latest_msg();
        move(cmd.setpoint_3d, cmd.mode, cmd.ref_frame);       

        delay(1); 
    }
}




void MotionModule::move(arma::vec setpoint_3d, CTRL_Mode mode, ReferenceFrame setpoint_ref_frame) { // default: setpoint frame is world frame
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
        case NSTDRD: trans_setpoint.type = CTRL::displacement;
                     rotat_setpoint.type = CTRL::displacement;
                     break;
        case NSTDRV: trans_setpoint.type = CTRL::displacement;
                     rotat_setpoint.type = CTRL::velocity;
                     break;
    }

    trans_setpoint.value = {setpoint_3d(0), setpoint_3d(1)};
    rotat_setpoint.value = setpoint_3d(2);


    // If CMD setpoint is described in World Reference Frame, we do a Homogeneouse Transformation
    if(setpoint_ref_frame == WorldFrame) {
        if(mode == TDRD || mode == TDRV || mode == NSTDRD || mode == NSTDRV) { // Position Control : Homo-transform a homgeneous POINT
        
            arma::vec bot_origin = robot_origin_w_sub.latest_msg();
            double bot_orien = sensor_sub.latest_msg().rotat_disp;

            /* The math trick here is we define body frame to be (bot_origin_x, bot_origin_y, bot_orien)
             * in which bot_origin_x/y are static, while bot_orien changes along with the moving robot.
             * This is meant to simplify things to avoid having a body frame that moves. So we in fact
             * have a stationary body frame that essentially only have the rotation transformation being meaningful
             */

            /* a not-so-obvious simplification was done by 
                * using bot_origin as the bot curr location */
            arma::mat A = wtb_homo_transform(bot_origin, bot_orien); // world to body homogeneous transformation

            // setpoint with respect to world reference frame
            arma::vec setpoint_w = {trans_setpoint.value(0), trans_setpoint.value(1), 1}; // homogeneous point end with a 1 (vector end with a 0)

            arma::vec setpoint_b = A * setpoint_w; // apply transformation to get the same point represented in the body frame

            // if division factor is approx. eq to zero
            if(std::fabs(setpoint_b(2)) < 0.000001) {
                setpoint_b(2) = 0.000001;
            }
            
            // update setpoint to the setpoint in robot's perspective
            trans_setpoint.value = {setpoint_b(0)/setpoint_b(2), setpoint_b(1)/setpoint_b(2)}; // the division is to divide the scaling factor, according to rules of homogeneous coord systems
            

            /* for rotational, we simply unify their zero orientations to avoid needing transformations */
        }
        else { // (Trans) Velocity Control : Homo-transform a homgeneous VECTOR
            arma::vec zero_vec = {0, 0};
            double bot_orien = sensor_sub.latest_msg().rotat_disp;
            arma::mat A = wtb_homo_transform(zero_vec, bot_orien);

            arma::vec setpoint_w = {trans_setpoint.value(0), trans_setpoint.value(1), 0}; // homogeneous vector end with a 0

            arma::vec setpoint_b = A * setpoint_w; // apply transformation to get the same point represented in the body frame

            // update setpoint to the setpoint in robot's perspective
            trans_setpoint.value = {setpoint_b(0), setpoint_b(1)}; // the computation here is identical to regular coordinate transformation, no need to divide the scaling factor   
        }
    }

    trans_setpoint_pub.publish(trans_setpoint);
    rotat_setpoint_pub.publish(rotat_setpoint);

    if(mode == NSTDRD || mode == NSTDRV) {
        no_slowdown_pub.publish(true);
    }
    else {
        no_slowdown_pub.publish(false);
    }
    
}

