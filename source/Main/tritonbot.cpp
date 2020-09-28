#include <iostream>
#include <armadillo>
#include "PubSubModule/thread_pool.hpp"
#include "Utility/boost_logger.hpp"
#include "ProtoGenerated/vFirmware_API.pb.h"
#include "Utility/systime.hpp"
#include "Utility/common.hpp"
#include "Config/config.hpp"

////////////////////////MODULES///////////////////////////
#include "MicroCtrlerClient/microctrler_client_module.hpp"
#include "MicroCtrlerClient/vfirm_client.hpp"
#include "EKF-Module/motion_ekf_module.hpp"
#include "EKF-Module/virtual_motion_ekf.hpp"
#include "ControlModule/control_module.hpp"
#include "ControlModule/pid_system.hpp"
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
    // ITPS::Publisher<VF_Commands> dummy_for_testing_only("vfirm-client", "commands");



    boost::shared_ptr<MicroCtrlerClientModule> uc_client_module(new VFirmClient());
    uc_client_module->run(thread_pool); // runs in a separate thread

    
    // /* vfirm client module unit test */
    // // -----------------------------------------
    // delay(500); //wait 500ms for vfirm_client_module to be ready
    // init_sensor_pub.publish(true); // signal the vfirm client to send init packet
    //
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




    // /* pseudo EKF module unit test */
    // // -----------------------------------------    
    // boost::shared_ptr<MotionEKF_Module> ekf_module (new VirtualMotionEKF());
    // ekf_module->run(thread_pool);


    // boost::thread sub1_thrd([]() {
    //     B_Log logger;
    //     logger.add_tag("SUBSCRIBER 1");
    //     ITPS::Subscriber<MotionEKF::MotionData> motion_data_sub("virtual-motion ekf", "motion prediction", 100);
    //     while(!motion_data_sub.subscribe());
    //     MotionEKF::MotionData motion_data;
        
    //     while(1) {
    //         std::ostringstream debug_out_stream;
    //         motion_data = motion_data_sub.pop_msg();
           
    //         debug_out_stream << motion_data.trans_disp << " " 
    //                          << motion_data.trans_vel << " "
    //                          << motion_data.rotat_disp << " "
    //                          << motion_data.rotat_vel;
    //         logger.log(Info, debug_out_stream.str());

    //     }
    // });

    // boost::thread sub2_thrd([]() {
    //     B_Log logger;
    //     logger.add_tag("SUBSCRIBER 2");
    //     ITPS::Subscriber<MotionEKF::MotionData> motion_data_sub("virtual-motion ekf", "motion prediction", 100);
    //     while(!motion_data_sub.subscribe());
    //     MotionEKF::MotionData motion_data;
        
    //     while(1) {
    //         std::ostringstream debug_out_stream;
    //         motion_data = motion_data_sub.pop_msg();
           
    //         debug_out_stream << motion_data.trans_disp << " " 
    //                          << motion_data.trans_vel << " "
    //                          << motion_data.rotat_disp << " "
    //                          << motion_data.rotat_vel;
    //         logger.log(Info, debug_out_stream.str());

    //     }
    // });

    // sub1_thrd.join();
    // sub2_thrd.join();

    // // -----------------------------------------


    /* PID system unit test*/
    // -----------------------------------------
    boost::shared_ptr<MotionEKF_Module> ekf_module (new VirtualMotionEKF());
    ekf_module->run(thread_pool);

    boost::shared_ptr<ControlModule> ctrl_module(new PID_System());
    ctrl_module->run(thread_pool);

    ITPS::Publisher<bool> dribbler_pub("CTRL", "dribbler");
    dribbler_pub.publish(false);
    ITPS::Publisher<arma::vec> kicker_pub("CTRL", "kicker");
    arma::vec zero_vec = {0, 0};
    kicker_pub.publish(zero_vec);

    boost::thread([]{
        ITPS::Publisher<bool> enable_signal_pub("safety", "enable"); // MQ Mode
        while(1) {
            enable_signal_pub.publish(true);
        }
    });

    boost::thread([]{
        ITPS::Publisher<CTRL::SetPoint<arma::vec>> trans_setpoint_pub("CTRL", "trans"); // Trivial Mode
        ITPS::Publisher<CTRL::SetPoint<float>> rotat_setpoint_pub("CTRL", "rotat"); // Trivial Mode

        
        CTRL::SetPoint<float> rotat_sp;
        rotat_sp.type = CTRL::displacement;

        CTRL::SetPoint<arma::vec> trans_sp;
        trans_sp.type = CTRL::velocity;

        arma::vec zero_vec = {0, 0};

        while(1) {
            rotat_sp.value = 30;
            rotat_setpoint_pub.publish(rotat_sp);

            trans_sp.value = zero_vec;
            trans_setpoint_pub.publish(trans_sp);
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
