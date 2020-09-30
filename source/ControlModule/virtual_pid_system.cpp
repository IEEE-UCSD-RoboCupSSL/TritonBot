#include "ControlModule/virtual_pid_system.hpp"
#include "ControlModule/pid_system.hpp"
#include "PubSubSystem/thread_pool.hpp"
#include "ProtoGenerated/vFirmware_API.pb.h"
#include "Utility/systime.hpp"
#include "Utility/boost_logger.hpp"
#include "Config/config.hpp"
#include "ControlModule/pid.hpp"
#include "EKF-Module/motion_ekf_module.hpp"
#include <armadillo>

Virtual_PID_System::Virtual_PID_System() : PID_System()
{}


void Virtual_PID_System::init_subscribers(void) {
    PID_System::init_subscribers();
}

MotionEKF::MotionData Virtual_PID_System::get_ekf_feedbacks(void) {
    MotionEKF::MotionData feedback_data = PID_System::get_ekf_feedbacks();
    /* because the velocity data calc from the simulator is not stable
     * here we sneaky-mask the velocity data by making them zero(s),
     * so the pid controller will forward the setpoint value directly, 
     * which delivers good enough performance on the simulator
     */

    feedback_data.rotat_vel = 0.00;
    feedback_data.trans_vel = {0.00, 0.00};
    return feedback_data;

}
