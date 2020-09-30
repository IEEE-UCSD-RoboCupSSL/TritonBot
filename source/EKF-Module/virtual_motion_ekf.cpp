#include "EKF-Module/virtual_motion_ekf.hpp"
#include "PubSubSystem/thread_pool.hpp"
#include "ProtoGenerated/vFirmware_API.pb.h"
#include "Utility/boost_logger.hpp"
#include "Config/config.hpp"


VirtualMotionEKF::VirtualMotionEKF() : MotionEKF_Module() {}


VirtualMotionEKF::~VirtualMotionEKF() {}


void VirtualMotionEKF::task(ThreadPool& thread_pool) {
    B_Log logger;
    logger.add_tag("PseudoMotionEKF Module");

    logger(Info) << "\033[0;32m Thread Started \033[0m";

    init_subscribers();

    VF_Data vf_data;
    MotionEKF::MotionData m_data;

    while(1) {
        vf_data = get_firmware_data();
        
        m_data.trans_disp = {vf_data.translational_displacement().x(), 
                             vf_data.translational_displacement().y()};

        m_data.trans_vel = {vf_data.translational_displacement().x(), 
                            vf_data.translational_displacement().y()};
            
        m_data.rotat_disp = vf_data.rotational_displacement();
        m_data.rotat_vel = vf_data.rotational_velocity();

        publish_motion_data(m_data);
        
    }
}