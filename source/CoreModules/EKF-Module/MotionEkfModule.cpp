#include "CoreModules/EKF-Module/MotionEkfModule.hpp"
#include "Config/Config.hpp"
#include "Misc/Utility/BoostLogger.hpp"
#include <ProtoGenerated/messages_robocup_ssl_wrapper.pb.h>
#include "Misc/PubSubSystem/ThreadPool.hpp"
#include "ProtoGenerated/vFirmware_API.pb.h"
#include "Misc/Utility/BoostLogger.hpp"
#include "Config/Config.hpp"
#include "Misc/Utility/Systime.hpp"

using namespace boost;
using namespace boost::asio;
using namespace boost::asio::ip;

static MotionEKF::MotionData default_md() {
    MotionEKF::MotionData rtn;
    arma::vec zero_vec = {0, 0};
    rtn.trans_disp = zero_vec;
    rtn.trans_vel = zero_vec;
    rtn.rotat_disp = 0.00;
    rtn.rotat_vel = 0.00;
    return rtn;
} 

MotionEKF_Module::MotionEKF_Module() : motion_data_pub("MotionEKF", "MotionData", default_md()),
                        firm_data_sub("FirmClient", "InternalSensorData", FIRM_DATA_MQ_SIZE) //construct with blocking mode
                        // ssl_data_sub("CMDListener", "GlobalSSLVisionData") // construct with nonblocking mode
{}

MotionEKF_Module::~MotionEKF_Module() {} 


void MotionEKF_Module::init_subscribers() {
    try {
        firm_data_sub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        // sslvison....
    }
    catch(std::exception& e) {
        B_Log logger;
        logger.add_tag("[motion_ekf_module.cpp]");
        logger.log(Error, e.what());
        std::exit(0);
    }

}


// To-do get ssl vision data
// ....() {}



VF_Data MotionEKF_Module::get_firmware_data() {
    return firm_data_sub.pop_msg();
}

void MotionEKF_Module::publish_motion_data(MotionData data) {
    motion_data_pub.publish(data);
}

/*  */

const bool is_blue_team_side = true;
const int vel_sample_period_ms = 50; // 50 ms
const double vel_max_thresh = 10000.00; // 1000

VirtualMotionEKF::VirtualMotionEKF() : MotionEKF_Module() {}


VirtualMotionEKF::~VirtualMotionEKF() = default;


[[noreturn]] void VirtualMotionEKF::task(ThreadPool& threadPool){
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

        /*
            logger.log(Info, "[virtual_motion_ekf] trans_disp_x_y: (" + std::to_string(vf_data.translational_displacement().x()) + ", " + std::to_string(vf_data.translational_displacement().y()) + ")");
            logger.log(Info, "[virtual_motion_ekf] trans_vel_x_y: (" + std::to_string(vf_data.translational_velocity().x()) + ", " + std::to_string(vf_data.translational_velocity().y()) + ")");
            logger.log(Info, "[virtual_motion_ekf] rotate_disp: (" + std::to_string(vf_data.rotational_displacement()) + ")");
            logger.log(Info, "[virtual_motion_ekf] rotate_vel: (" + std::to_string(vf_data.rotational_velocity()) + ")");
        */

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
