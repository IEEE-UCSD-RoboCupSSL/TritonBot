#pragma once
#include "PubSubSystem/module.hpp"
#include <armadillo>
#include "ProtoGenerated/vFirmware_API.pb.h"



class MotionEKF_Module : public Module {
    public:

        struct MotionData {
            arma::vec trans_disp;
            arma::vec trans_vel;
            float rotat_disp;
            float rotat_vel; 
        };

        MotionEKF_Module();
        virtual ~MotionEKF_Module();

        
        virtual void task(ThreadPool& thread_pool) = 0;


    protected:
        virtual void init_subscribers();

        // To-do SSL_VisionData get_.....
        VF_Data get_firmware_data();
        void publish_motion_data(MotionData data);

    private:
        ITPS::NonBlockingPublisher<MotionEKF_Module::MotionData> motion_data_pub;
        ITPS::BlockingSubscriber<VF_Data> firm_data_sub; /* internal sensor data, which should be
                                                  * sampled faster than the ssl vision data
                                                  */ 
        // ITPS::Subscriber<SSL_VisionData>  (trivial mode) .... To-do  


};

// alias
using MotionEKF = MotionEKF_Module;