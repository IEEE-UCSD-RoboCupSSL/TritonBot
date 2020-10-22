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
#include "EKF-Module/ball_ekf_module.hpp"
#include "EKF-Module/virtual_ball_ekf.hpp"
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

    ThreadPool thread_pool(THREAD_POOL_SIZE); // pre-allocate # threads in a pool
    ITPS::NonBlockingPublisher<bool> init_sensor_pub("vfirm-client", "re/init sensors", false);


    while(true){

    }

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
