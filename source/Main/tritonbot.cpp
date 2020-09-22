#include <iostream>
#include "PubSubModule/thread_pool.hpp"
#include "MicroCtrlerInterface/vfirm_client.hpp"
#include "Utility/boost_logger.hpp"
#include "ProtoGenerated/vFirmware_API.pb.h"
#include "Utility/systime.hpp"
#include "Utility/common.hpp"

int main(int arc, char *argv[]) {

    B_Log::static_init();
    B_Log::set_shorter_format();
    B_Log::sink->set_filter(severity >= Info);
    B_Log logger;

/* thread pool version */
    ThreadPool thread_pool(20); // pre-allocate 10 threads in a pool
    ITPS::Publisher<bool> init_sensor_pub("vfirm-client", "init sensors");
    ITPS::Publisher<VF_Commands> dummy_for_testing_only("vfirm-client", "commands");

    VFirmClient vfirm_client_module;
    vfirm_client_module.run(thread_pool); // runs in a separate thread

    delay(500); //wait 500ms for vfirm_client_module to be ready
    init_sensor_pub.publish(true); // signal the vfirm client to send init packet

    /* vfirm client module unit test */
    // -----------------------------------------
    
    ITPS::Subscriber<VF_Data> vfirm_client_data_sub("vfirm-client", "data", 100);
    while(!vfirm_client_data_sub.subscribe());
    VF_Data curr_data;
    
    while(1)
    {
        curr_data = vfirm_client_data_sub.pop_msg();
    
        logger.log( Info, "Trans_Dis: " + repr(curr_data.translational_displacement().x()) + ' ' + repr(curr_data.translational_displacement().y()));
        logger.log( Info, "Trans_Vel:" + repr(curr_data.translational_velocity().x()) + ' ' + repr(curr_data.translational_velocity().y()));
        logger.log( Info, "Rot_Dis:" + repr(curr_data.rotational_displacement()));
        logger.log( Info, "Rot_Vel:" + repr(curr_data.rotational_velocity()) + "\n :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) ");
    }
    
    // -----------------------------------------

    while(1);

    return 0;
}