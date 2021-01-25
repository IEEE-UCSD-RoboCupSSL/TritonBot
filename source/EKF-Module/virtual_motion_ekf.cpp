#include <ProtoGenerated/messages_robocup_ssl_wrapper.pb.h>
#include "EKF-Module/virtual_motion_ekf.hpp"
#include "PubSubSystem/thread_pool.hpp"
#include "ProtoGenerated/vFirmware_API.pb.h"
#include "Utility/boost_logger.hpp"
#include "Config/config.hpp"
#include "Utility/systime.hpp"

using namespace boost;
using namespace boost::asio;
using namespace boost::asio::ip;

const bool is_blue_team_side = true;
const int vel_sample_period_ms = 50; // 50 ms
const double vel_max_thresh = 10000.00; // 1000

VirtualMotionEKF::VirtualMotionEKF() : MotionEKF_Module() {}


VirtualMotionEKF::~VirtualMotionEKF() = default;


[[noreturn]] void VirtualMotionEKF::task(ThreadPool& thread_pool){
    B_Log logger;
    logger.add_tag("PseudoMotionEKF Module");

    logger(Info) << "\033[0;32m Thread Started \033[0m";

    init_subscribers();
    logger(Info) << "\033[0;32m Initialized \033[0m";

    VF_Data vf_data;
    MotionEKF::MotionData m_data;

    while(true) { // has delay (good for reducing high CPU usage)
        // firm data is in body(not global) frame!
        vf_data = get_firmware_data();

        if(false){
            logger.log(Info, "[virtual_motion_ekf] trans_disp_x_y: (" + std::to_string(vf_data.translational_displacement().x()) + ", " + std::to_string(vf_data.translational_displacement().y()) + ")");
            logger.log(Info, "[virtual_motion_ekf] trans_vel_x_y: (" + std::to_string(vf_data.translational_velocity().x()) + ", " + std::to_string(vf_data.translational_velocity().y()) + ")");
            logger.log(Info, "[virtual_motion_ekf] rotate_disp: (" + std::to_string(vf_data.rotational_displacement()) + ")");
            logger.log(Info, "[virtual_motion_ekf] rotate_vel: (" + std::to_string(vf_data.rotational_velocity()) + ")");
        }
        
        m_data.trans_disp = {vf_data.translational_displacement().x(), 
                             vf_data.translational_displacement().y()};

        m_data.trans_vel = {vf_data.translational_velocity().x(),
                            vf_data.translational_velocity().y()};

        // Rotational data are all in world frame
        m_data.rotat_disp = vf_data.rotational_displacement();
        m_data.rotat_vel = vf_data.rotational_velocity();

        publish_motion_data(m_data);

        delay(1);
    }
}
