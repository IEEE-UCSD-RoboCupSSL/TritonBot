#include <iostream>
#include <armadillo>
#include "PubSubSystem/thread_pool.hpp"
#include "Utility/boost_logger.hpp"
#include "ProtoGenerated/vFirmware_API.pb.h"
#include "Utility/systime.hpp"
#include "Utility/common.hpp"
#include "Config/config.hpp"

////////////////////////MODULES///////////////////////////
#include "FirmClientModule/firmware_client_module.hpp"
#include "FirmClientModule/vfirm_client.hpp"
#include "EKF-Module/motion_ekf_module.hpp"
#include "EKF-Module/virtual_motion_ekf.hpp"
#include "ControlModule/control_module.hpp"
#include "ControlModule/pid_system.hpp"
#include "ControlModule/virtual_pid_system.hpp"
#include "MotionModule/motion_module.hpp"
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
    // ITPS::Publisher<VF_Commands> dummy_for_testing_only("FirmClient", "Commands");



    boost::shared_ptr<FirmClientModule> uc_client_module(new VFirmClient());
    uc_client_module->run(thread_pool); // runs in a separate thread


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    // /* vfirm client module unit test */
    // // -----------------------------------------
    // delay(500); //wait 500ms for vfirm_client_module to be ready
    // init_sensor_pub.publish(true); // signal the vfirm client to send init packet
    //
    // ITPS::Subscriber<VF_Data> vfirm_client_data_sub("FirmClient", "InternalSensorData", 100);
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


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    // /* pseudo EKF module unit test */
    // // -----------------------------------------    
    // boost::shared_ptr<MotionEKF_Module> ekf_module (new VirtualMotionEKF());
    // ekf_module->run(thread_pool);


    // boost::thread sub1_thrd([]() {
    //     B_Log logger;
    //     logger.add_tag("SUBSCRIBER 1");
    //     ITPS::Subscriber<MotionEKF::MotionData> motion_data_sub("MotionEKF", "MotionData", 100);
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
    //     ITPS::Subscriber<MotionEKF::MotionData> motion_data_sub("MotionEKF", "MotionData", 100);
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


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // /* PID system unit test*/
    // // -----------------------------------------
    // boost::shared_ptr<MotionEKF_Module> ekf_module (new VirtualMotionEKF());
    // ekf_module->run(thread_pool);

    // boost::shared_ptr<ControlModule> ctrl_module(new Virtual_PID_System());
    // ctrl_module->run(thread_pool);

    // ITPS::Publisher<bool> dribbler_pub("AI CMD", "Dribbler");
    // dribbler_pub.publish(false);
    // ITPS::Publisher<arma::vec> kicker_pub("AI CMD", "Kicker");
    // arma::vec zero_vec = {0, 0};
    // kicker_pub.publish(zero_vec);


    // delay(500); //wait 500ms for vfirm_client_module to be ready
    // init_sensor_pub.publish(true); // signal the vfirm client to send init packet

    // boost::thread([]{
    //     ITPS::Publisher<bool> enable_signal_pub("AI CMD", "SafetyEnable"); // MQ Mode
    //     while(1) {
    //         enable_signal_pub.publish(true);
    //     }
    // });

    // boost::thread([]{
    //     ITPS::Publisher<CTRL::SetPoint<arma::vec>> trans_setpoint_pub("AI CMD", "Trans"); // Trivial Mode
    //     ITPS::Publisher<CTRL::SetPoint<float>> rotat_setpoint_pub("AI CMD", "Rotat"); // Trivial Mode
    //     delay(800); // wait for other threads are ready
        
    //     CTRL::SetPoint<float> rotat_sp;
    //     CTRL::SetPoint<arma::vec> trans_sp;

    //     int refresh_origin_cnt = 0;
    //     bool refresh;

    //     bool DorV;
    //     double x, y;

    //     ITPS::Publisher<PID_System::PID_Constants> pid_const_pub("PID", "Constants");

    //     PID_System::PID_Constants pid_consts;
    //     pid_consts.RD_Kd = PID_RD_KD;
    //     pid_consts.RD_Ki = PID_RD_KI;
    //     pid_consts.RD_Kp = PID_RD_KP; 
    //     pid_consts.RV_Ki = PID_RV_KI;
    //     pid_consts.RV_Kp = PID_RV_KP;
    //     pid_consts.RV_Kd = PID_RV_KD;
    //     pid_consts.TD_Kd = PID_TD_KD;
    //     pid_consts.TD_Ki = PID_TD_KI;
    //     pid_consts.TD_Kp = PID_TD_KP;
    //     pid_consts.TV_Kd = PID_TV_KD;
    //     pid_consts.TV_Ki = PID_TV_KI;
    //     pid_consts.TV_Kp = PID_TV_KP;
        
    //     // std::cout << ">>> TD: Kp, Ki, Kd" << std::endl;
    //     // std::cin >> pid_consts.TD_Kp >> pid_consts.TD_Ki >> pid_consts.TD_Kd;
    //     // pid_const_pub.publish(pid_consts);


    //     while(1) {

    //         std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> " << std::endl;
    //         std::cout << "Rotation Disp [1] ? or Vel [0]  |  SetPoint [x]" << std::endl;
    //         std::cin >> DorV >> x;
    //         rotat_sp.type = DorV ? CTRL::displacement : CTRL::velocity;
    //         rotat_sp.value = (float)x;

    //         std::cout << "Translation Disp [1] ? or Vel [0] | SetPoint [x, y]" << std::endl;
    //         std::cin >> DorV >> x >> y;
    //         trans_sp.type = DorV ? CTRL::displacement : CTRL::velocity;
    //         trans_sp.value = {x, y};
    //         std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< " << std::endl;


    //         rotat_setpoint_pub.publish(rotat_sp);
    //         trans_setpoint_pub.publish(trans_sp);
    //         delay(1000);

    //     }
        
    // });

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /* Motion Module unit test */ // motion is a wrapper of control module with math to transform coordinate systems
    boost::shared_ptr<MotionEKF_Module> ekf_module (new VirtualMotionEKF());
    ekf_module->run(thread_pool);

    boost::shared_ptr<ControlModule> ctrl_module(new Virtual_PID_System());
    ctrl_module->run(thread_pool);

    boost::shared_ptr<MotionModule> motion_module(new MotionModule());
    motion_module->run(thread_pool);

    ITPS::Publisher<bool> dribbler_pub("AI CMD", "Dribbler");
    dribbler_pub.publish(false);
    ITPS::Publisher<arma::vec> kicker_pub("AI CMD", "Kicker");
    arma::vec zero_vec = {0, 0};
    kicker_pub.publish(zero_vec);

    ITPS::Publisher<PID_System::PID_Constants> pid_const_pub("PID", "Constants");

    delay(500); //wait 500ms for vfirm_client_module to be ready
    init_sensor_pub.publish(true); // signal the vfirm client to send init packet

    boost::thread([]{
        ITPS::Publisher<bool> enable_signal_pub("AI CMD", "SafetyEnable"); // MQ Mode
        while(1) {
            enable_signal_pub.publish(true);
        }
    });

    boost::thread([&]{
        ITPS::Publisher< arma::vec > robot_origin_w_pub("ConnectionInit", "RobotOrigin(WorldFrame)"); 
        ITPS::Publisher< Motion::MotionCMD > command_pub("CMD Server", "MotionCMD");
        delay(1200); // wait for everything is started
        arma::vec origin = {0, 0};
        std::cout << "Enter robot origin <x, y>" << std::endl;
        std::cin >> origin(0) >> origin(1);
        robot_origin_w_pub.publish(origin);
        init_sensor_pub.publish(true); 
        Motion::MotionCMD cmd;

        while(1) {
            cmd.mode = Motion::CTRL_Mode::TDRD;
            cmd.ref_frame = Motion::ReferenceFrame::WorldFrame;
            cmd.setpoint_3d = {0, 0, 0};
            std::cout << "cmd3D: <x, y, theta>" << std::endl;
            std::cin >> cmd.setpoint_3d(0) >> cmd.setpoint_3d(1) >> cmd.setpoint_3d(2); 
            command_pub.publish(cmd);
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
