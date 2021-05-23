#include "CoreModules/EKF-Module/MotionEkfModule.hpp"
#include "Config/Config.hpp"
#include "Misc/Utility/BoostLogger.hpp"
#include <ProtoGenerated/messages_robocup_ssl_wrapper.pb.h>
#include "Misc/PubSubSystem/ThreadPool.hpp"
#include "ProtoGenerated/vFirmware_API.pb.h"
#include "Misc/Utility/BoostLogger.hpp"
#include "Config/Config.hpp"
#include "Misc/Utility/ClockUtil.hpp"

using namespace boost;
using namespace boost::asio;
using namespace boost::asio::ip;

static BotData default_md() {
    BotData rtn;
    arma::vec zero_vec = {0, 0};
    rtn.pos = zero_vec;
    rtn.vel = zero_vec;
    rtn.ang = 0.00;
    rtn.angVel = 0.00;
    return rtn;
} 

MotionEKF_Module::MotionEKF_Module() : motion_data_pub("MotionEKF", "BotProcessedData", default_md()),
                        firm_data_sub("FirmClient", "InternalSensorData") // DELETE-ME //construct with blocking mode
                        // ssl_data_sub("CMDListener", "GlobalSSLVisionData") // construct with nonblocking mode
{}

MotionEKF_Module::~MotionEKF_Module() {} 


void MotionEKF_Module::init_subscribers() {
    try {
        firm_data_sub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        // sslvison....
    }
    catch(std::exception& e) {
        BLogger logger;
        logger.addTag("[motion_ekf_module.cpp]");
        logger.log(Error, e.what());
        std::exit(0);
    }

}


// To-do get ssl vision data
// ....() {}



VF_Data MotionEKF_Module::get_firmware_data() {
    return firm_data_sub.getMsg();
}

void MotionEKF_Module::publish_motion_data(BotData data) {
    motion_data_pub.publish(data);
}

/*  */

const bool is_blue_team_side = true;
const int vel_sample_period_ms = 50; // 50 ms
const double vel_max_thresh = 10000.00; // 1000

VirtualMotionEKF::VirtualMotionEKF() : MotionEKF_Module() {}


VirtualMotionEKF::~VirtualMotionEKF() = default;


[[noreturn]] void VirtualMotionEKF::task(ThreadPool& threadPool){
    BLogger logger;
    logger.addTag("PseudoMotionEKF Module");

    logger(Info) << "\033[0;32m Thread Started \033[0m";

    init_subscribers();
    logger(Info) << "\033[0;32m Initialized \033[0m";

    VF_Data vf_data;
    BotData m_data;

    while(true) { // has delay (good for reducing high CPU usage)
        // firm data is in body(not global) frame!
        vf_data = get_firmware_data();

        /*
            logger.log(Info, "[virtual_motion_ekf] trans_disp_x_y: (" + std::to_string(vf_data.translational_displacement().x()) + ", " + std::to_string(vf_data.translational_displacement().y()) + ")");
            logger.log(Info, "[virtual_motion_ekf] trans_vel_x_y: (" + std::to_string(vf_data.translational_velocity().x()) + ", " + std::to_string(vf_data.translational_velocity().y()) + ")");
            logger.log(Info, "[virtual_motion_ekf] rotate_disp: (" + std::to_string(vf_data.rotational_displacement()) + ")");
            logger.log(Info, "[virtual_motion_ekf] rotate_vel: (" + std::to_string(vf_data.rotational_velocity()) + ")");
        */

        m_data.pos = {vf_data.translational_displacement().x(),
                             vf_data.translational_displacement().y()};

        m_data.vel = {vf_data.translational_velocity().x(),
                            vf_data.translational_velocity().y()};

        // Rotational data are all in world frame
        m_data.ang = vf_data.rotational_displacement();
        m_data.angVel = vf_data.rotational_velocity();

        publish_motion_data(m_data);

        delay(1);
    }
}
