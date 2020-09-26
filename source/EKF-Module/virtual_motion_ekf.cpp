#include "EKF-Module/virtual_motion_ekf.hpp"
#include "PubSubModule/thread_pool.hpp"
#include "ProtoGenerated/vFirmware_API.pb.h"
#include "Utility/boost_logger.hpp"
#include "Config/config.hpp"



void VirtualMotionEFK::task(ThreadPool& thread_pool) {
    B_Log logger;
    logger.add_tag("PseudoMotionEKF Module");

    logger(Info) << "\033[0;32m Thread Started \033[0m";

    ITPS::Publisher<VirtualMotionEFK::MotionData> motion_data_pub("virtual-motion ekf", "motion prediction");
    ITPS::Subscriber<VF_Data> vf_data_sub("vfirm-client", "data", VF_DATA_MQ_SIZE); //construct with a message queue as buffer    
    while(!vf_data_sub.subscribe());

    VF_Data vf_data;
    VirtualMotionEFK::MotionData m_data;

    while(1) {
        vf_data = vf_data_sub.pop_msg();
        
        m_data.trans_disp = {vf_data.translational_displacement().x(), 
                             vf_data.translational_displacement().y()};

        m_data.trans_vel = {vf_data.translational_displacement().x(), 
                            vf_data.translational_displacement().y()};
            
        m_data.rotat_disp = vf_data.rotational_displacement();
        m_data.rotat_vel = vf_data.rotational_velocity();

        motion_data_pub.publish(m_data);
        
    }
}