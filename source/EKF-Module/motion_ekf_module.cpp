#include "EKF-Module/motion_ekf_module.hpp"
#include "Config/config.hpp"


MotionEKF_Module::MotionEKF_Module() : motion_data_pub("MotionEKF", "MotionData"),
                        firm_data_sub("FirmClient", "InternalSensorData", FIRM_DATA_MQ_SIZE) //construct with MQ Mode
                        // ssl_data_sub("CMDListener", "GlobalSSLVisionData") // construct with trivial mode
{}

MotionEKF_Module::~MotionEKF_Module() {} 


void MotionEKF_Module::init_subscribers() {
    while(!firm_data_sub.subscribe());

    // while(!sslvison....)

}


// To-do get ssl vision data
// ....() {}



VF_Data MotionEKF_Module::get_firmware_data() {
    return firm_data_sub.pop_msg();
}

void MotionEKF_Module::publish_motion_data(MotionData data) {
    motion_data_pub.publish(data);
}