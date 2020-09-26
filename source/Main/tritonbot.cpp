#include <iostream>
#include "PubSubModule/thread_pool.hpp"
#include "Utility/boost_logger.hpp"
#include "ProtoGenerated/vFirmware_API.pb.h"
#include "Utility/systime.hpp"
#include "Utility/common.hpp"
#include "Config/config.hpp"

////////////////////////MODULES///////////////////////////
#include "MicroCtrlerInterface/vfirm_client.hpp"
#include "EKF-Module/virtual_motion_ekf.hpp"
//////////////////////////////////////////////////////////


std::ostream& operator<<(std::ostream& os, const arma::vec& v);

int main(int arc, char *argv[]) {

    B_Log::static_init();
    B_Log::set_shorter_format();
    // B_Log::sink->set_filter(severity >= Debug && tag_attr == "VFirmClient Module");
    B_Log::sink->set_filter(severity >= Info);
    
    
    B_Log logger;


    ThreadPool thread_pool(THREAD_POOL_SIZE); // pre-allocate 10 threads in a pool
    ITPS::Publisher<bool> init_sensor_pub("vfirm-client", "re/init sensors");
    ITPS::Publisher<VF_Commands> dummy_for_testing_only("vfirm-client", "commands");

    VFirmClient vfirm_client_module;
    vfirm_client_module.run(thread_pool); // runs in a separate thread

    delay(500); //wait 500ms for vfirm_client_module to be ready
    init_sensor_pub.publish(true); // signal the vfirm client to send init packet

    // /* vfirm client module unit test */
    // // -----------------------------------------
    
    // ITPS::Subscriber<VF_Data> vfirm_client_data_sub("vfirm-client", "data", 100);
    // while(!vfirm_client_data_sub.subscribe());
    // VF_Data curr_data;
    
    // while(1)
    // {
    //     curr_data = vfirm_client_data_sub.pop_msg();
    
    //     logger.log( Info, "Trans_Dis: " + repr(curr_data.translational_displacement().x()) + ' ' + repr(curr_data.translational_displacement().y()));
    //     logger.log( Info, "Trans_Vel:" + repr(curr_data.translational_velocity().x()) + ' ' + repr(curr_data.translational_velocity().y()));
    //     logger.log( Info, "Rot_Dis:" + repr(curr_data.rotational_displacement()));
    //     logger.log( Info, "Rot_Vel:" + repr(curr_data.rotational_velocity()) + "\n :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) ");
    // }
    
    // // -----------------------------------------




    /* pseudo EKF module unit test */
    // -----------------------------------------    
    VirtualMotionEFK pseudo_EKF_module;
    pseudo_EKF_module.run(thread_pool);

    boost::thread([]() {
        B_Log logger;
        logger.add_tag("SUBSCRIBER 1");
        ITPS::Subscriber<VirtualMotionEFK::MotionData> motion_data_sub("virtual-motion ekf", "motion prediction", 100);
        while(!motion_data_sub.subscribe());
        VirtualMotionEFK::MotionData motion_data;
        
        while(1) {
            std::ostringstream debug_out_stream;
            motion_data = motion_data_sub.pop_msg();
           
            debug_out_stream << motion_data.trans_disp << " " 
                             << motion_data.trans_vel << " "
                             << motion_data.rotat_disp << " "
                             << motion_data.rotat_vel;
            logger.log(Info, debug_out_stream.str());

        }
    });

    boost::thread([]() {
        B_Log logger;
        logger.add_tag("SUBSCRIBER 2");
        ITPS::Subscriber<VirtualMotionEFK::MotionData> motion_data_sub("virtual-motion ekf", "motion prediction", 100);
        while(!motion_data_sub.subscribe());
        VirtualMotionEFK::MotionData motion_data;
        
        while(1) {
            std::ostringstream debug_out_stream;
            motion_data = motion_data_sub.pop_msg();
           
            debug_out_stream << motion_data.trans_disp << " " 
                             << motion_data.trans_vel << " "
                             << motion_data.rotat_disp << " "
                             << motion_data.rotat_vel;
            logger.log(Info, debug_out_stream.str());

        }
    });

    // -----------------------------------------

    while(1);

    return 0;
}

std::ostream& operator<<(std::ostream& os, const arma::vec& v)
{
    char fmt_str[30]; // not so important size, greater than printed str size is fine, use magic number here 
    int num_rows = arma::size(v).n_rows;
    os << "<";
    for(int i = 0; i < num_rows; i++) {
        sprintf(fmt_str, "%8.3lf", v(i));
        os << std::string(fmt_str);
        if(i != num_rows - 1) os << ", ";
    }
    os << ">";
    return os;
}