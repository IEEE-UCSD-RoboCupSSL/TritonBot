#include "EKF-Module/motion_ekf_module.hpp"
#include "Config/config.hpp"
#include "Utility/boost_logger.hpp"

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
        logger.log(Error, e.what());
        while(1);
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