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
#include "MotionModule/motion_module.hpp"
#include "RemoteServers/ConnectionServer/connection_server_module.hpp"
#include "RemoteServers/RemoteCMDServer/cmd_server_module.hpp"
//////////////////////////////////////////////////////////

std::ostream& operator<<(std::ostream& os, const arma::vec& v);

int main(int arc, char *argv[]) {

    

    B_Log::static_init();
    B_Log::set_shorter_format();
    // B_Log::sink->set_filter(severity >= Debug && tag_attr == "VFirmClient Module");
    B_Log::sink->set_filter(severity >= Info);
    
    B_Log logger;

    ThreadPool thread_pool(THREAD_POOL_SIZE); // pre-allocate 10 threads in a pool
    ITPS::NonBlockingPublisher<bool> init_sensor_pub("vfirm-client", "re/init sensors", false);

    
    /* Connection Server Unit Test */
    // boost::shared_ptr<ConnectionServerModule> cs_module(new ConnectionServer());
    // cs_module->run(thread_pool);
    // while(1);

    /* CMD Server Unit Test */
    boost::shared_ptr<CMDServerModule> cmd_server_module(new CMDServer());
    cmd_server_module->run(thread_pool);
    while(1);


    boost::shared_ptr<FirmClientModule> uc_client_module(new VFirmClient());
    uc_client_module->run(thread_pool); // runs in a separate thread



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // /* PID system unit test*/
    // // -----------------------------------------
    // boost::shared_ptr<MotionEKF_Module> ekf_module (new VirtualMotionEKF());
    // ekf_module->run(thread_pool);

    // boost::shared_ptr<ControlModule> ctrl_module(new PID_System());
    // ctrl_module->run(thread_pool);

    // ITPS::NonBlockingPublisher<bool> dribbler_pub("AI CMD", "Dribbler", false);
    // dribbler_pub.publish(false);
    // ITPS::NonBlockingPublisher<arma::vec> kicker_pub("AI CMD", "Kicker", zero_vec_2d());
    // arma::vec zero_vec = {0, 0};
    // kicker_pub.publish(zero_vec);


    // delay(500); //wait 500ms for vfirm_client_module to be ready
    // init_sensor_pub.publish(true); // signal the vfirm client to send init packet

    // boost::thread([]{
    //     ITPS::BlockingPublisher<bool> enable_signal_pub("AI CMD", "SafetyEnable"); // MQ Mode
    //     while(1) {
    //         enable_signal_pub.publish(true);
    //     }
    // });

    // boost::thread([]{
    //     CTRL::SetPoint<float> rotat_sp;
    //     CTRL::SetPoint<arma::vec> trans_sp;
    //     rotat_sp.type = CTRL::velocity;
    //     rotat_sp.value = 0.00;
    //     trans_sp.type = CTRL::velocity;
    //     trans_sp.value = zero_vec_2d();
    //     ITPS::NonBlockingPublisher<CTRL::SetPoint<arma::vec>> trans_setpoint_pub("AI CMD", "Trans", trans_sp); 
    //     ITPS::NonBlockingPublisher<CTRL::SetPoint<float>> rotat_setpoint_pub("AI CMD", "Rotat", rotat_sp); 
    //     delay(800); // wait for other threads are ready
        

    //     int refresh_origin_cnt = 0;
    //     bool refresh;

    //     bool DorV;
    //     double x, y;

    //     PID_System::PID_Constants pid_consts;
    //     pid_consts.RD_Kp = PID_RD_KP;   pid_consts.RD_Ki = PID_RD_KI;   pid_consts.RD_Kd = PID_RD_KD;
    //     pid_consts.TD_Kp = PID_TD_KP;   pid_consts.TD_Ki = PID_TD_KI;   pid_consts.TD_Kd = PID_TD_KD;
    //     ITPS::NonBlockingPublisher<PID_System::PID_Constants> pid_const_pub("PID", "Constants", pid_consts);
        
    //     // std::cout << ">>> TD: Kp, Ki, Kd" << std::endl;
    //     // std::cin >> pid_consts.TD_Kp >> pid_consts.TD_Ki >> pid_consts.TD_Kd;
    //     // pid_const_pub.publish(pid_consts);

    //     while(1) {

    //         // std::cout << ">>> RD: Kp, Ki, Kd" << std::endl;
    //         // std::cin >> pid_consts.RD_Kp >> pid_consts.RD_Ki >> pid_consts.RD_Kd;
    //         // pid_const_pub.publish(pid_consts);
        
    //         std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> " << std::endl;
    //         std::cout << "Rotation Disp [1] ? or Vel [0]  |  SetPoint [x]" << std::endl;
    //         std::cin >> DorV >> x;
    //         rotat_sp.type = DorV ? CTRL::displacement : CTRL::velocity;
    //         rotat_sp.value = (float)x;

    //         std::cout << "Translation Vel SetPoint [x, y]" << std::endl;
    //         std::cin >> x >> y;
    //         trans_sp.type = CTRL::velocity;
    //         trans_sp.value = {x, y};
    //         std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< " << std::endl;

    //         rotat_setpoint_pub.publish(rotat_sp);
    //         trans_setpoint_pub.publish(trans_sp);
    //         delay(3000);

    //         rotat_sp.type = CTRL::velocity;
    //         rotat_sp.value = 0;
    //         trans_sp.type = CTRL::velocity;
    //         trans_sp.value = {0, 0};
    //         rotat_setpoint_pub.publish(rotat_sp);
    //         trans_setpoint_pub.publish(trans_sp);

    //     }
        
    // });

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /* Motion Module unit test */ // motion is a wrapper of control module with math to transform coordinate systems
    boost::shared_ptr<MotionEKF_Module> ekf_module (new VirtualMotionEKF());
    ekf_module->run(thread_pool);

    boost::shared_ptr<ControlModule> ctrl_module(new PID_System());
    ctrl_module->run(thread_pool);

    boost::shared_ptr<MotionModule> motion_module(new MotionModule());
    motion_module->run(thread_pool);

    ITPS::NonBlockingPublisher<bool> dribbler_pub("AI CMD", "Dribbler", false);
    dribbler_pub.publish(false);
    arma::vec zero_vec = {0, 0};
    ITPS::NonBlockingPublisher<arma::vec> kicker_pub("AI CMD", "Kicker", zero_vec);
    kicker_pub.publish(zero_vec);


    PID_System::PID_Constants pid_consts;
    pid_consts.RD_Kp = PID_RD_KP;   pid_consts.RD_Ki = PID_RD_KI;   pid_consts.RD_Kd = PID_RD_KD;
    pid_consts.TD_Kp = PID_TD_KP;   pid_consts.TD_Ki = PID_TD_KI;   pid_consts.TD_Kd = PID_TD_KD;
    ITPS::NonBlockingPublisher<PID_System::PID_Constants> pid_const_pub("PID", "Constants", pid_consts);

    delay(500); //wait 500ms for vfirm_client_module to be ready
    init_sensor_pub.publish(true); // signal the vfirm client to send init packet

    boost::thread([]{
        ITPS::BlockingPublisher<bool> enable_signal_pub("AI CMD", "SafetyEnable"); // MQ Mode
        while(1) {
            enable_signal_pub.publish(true);
        }
    });

    boost::thread([&]{
        arma::vec zero_vec = {0, 0};
        ITPS::NonBlockingPublisher< arma::vec > robot_origin_w_pub("ConnectionInit", "RobotOrigin(WorldFrame)", zero_vec); 

        Motion::MotionCMD default_cmd;
        default_cmd.setpoint_3d = {0, 0, 0};
        default_cmd.mode = Motion::CTRL_Mode::TVRV;
        default_cmd.ref_frame = Motion::ReferenceFrame::BodyFrame;
        ITPS::NonBlockingPublisher< Motion::MotionCMD > command_pub("CMD Server", "MotionCMD", default_cmd);
        
        
        delay(1200); // wait for everything is started
        arma::vec origin = {0, 0};
        std::cout << "Enter robot origin <x, y>" << std::endl;
        std::cin >> origin(0) >> origin(1);
        robot_origin_w_pub.publish(origin);
        init_sensor_pub.publish(true); 
        Motion::MotionCMD cmd;
        Motion::ReferenceFrame frame;
        bool is_world_frame;
        int mode_idx;
        std::cout << "Reference Frame: World[1] or RobotBody[0]" << std::endl;
        std::cin >> is_world_frame;
        frame = is_world_frame ? Motion::ReferenceFrame::WorldFrame : Motion::ReferenceFrame::BodyFrame;
        double angle;
        while(1) {
            std::cout << "Ctrl Mode: TDRD[0] TDRV[1] TVRD[2] TVRV[3]" << std::endl;
            std::cin >> mode_idx;
            cmd.mode = static_cast<Motion::CTRL_Mode>(mode_idx);
            cmd.ref_frame = frame;
            cmd.setpoint_3d = {0, 0, 0};
            std::cout << "cmd3D: <x, y, theta>" << std::endl;
            std::cin >> cmd.setpoint_3d(0) >> cmd.setpoint_3d(1) >> cmd.setpoint_3d(2); 
            command_pub.publish(cmd);
            // std::cin >> angle;
            // cmd.mode = static_cast<Motion::CTRL_Mode>(0);
            // cmd.ref_frame = frame;
            // cmd.setpoint_3d = {0, 1000, angle};
            // command_pub.publish(cmd);
            // delay(1000);
            // cmd.mode = static_cast<Motion::CTRL_Mode>(0);
            // cmd.ref_frame = frame;
            // cmd.setpoint_3d = {0, 2000, angle*2};
            // command_pub.publish(cmd);
            // delay(1000);
            // cmd.mode = static_cast<Motion::CTRL_Mode>(0);
            // cmd.ref_frame = frame;
            // cmd.setpoint_3d = {0, 3000, angle*3};
            // command_pub.publish(cmd);
            // delay(1000);
            // cmd.mode = static_cast<Motion::CTRL_Mode>(0);
            // cmd.ref_frame = frame;
            // cmd.setpoint_3d = {0, 4000, angle*4};
            // command_pub.publish(cmd);
            // delay(1000);
        
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


// Ignore this part, just a backup
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
     // ITPS::Publisher<VF_Commands> dummy_for_testing_only("FirmClient", "Commands");
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

//