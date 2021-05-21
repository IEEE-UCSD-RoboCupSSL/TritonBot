#pragma once
#include "Misc/PubSubSystem/Module.hpp"
#include <armadillo>
#include "ProtoGenerated/vFirmware_API.pb.h"


/*  */

class MotionEKF_Module : public Module {
    public:

        /* Motion frame: when facing yellow gate from blue gate, it is the positive x direction. The positive y direction
         * is the positive x direction rotated counter-clockwise by 90 degree */
        struct BotData {
            arma::vec pos;
            arma::vec vel;
            float ang;   
            float angVel; 
        };

        MotionEKF_Module();
        virtual ~MotionEKF_Module();

        
        virtual void task(ThreadPool& threadPool) = 0;


    protected:
        virtual void init_subscribers();

        // To-do SSL_VisionData get_.....
        VF_Data get_firmware_data();
        void publish_motion_data(BotData data);

    private:
        ITPS::FieldPublisher<MotionEKF_Module::BotData> motion_data_pub;
        ITPS::MQSubscriber<VF_Data> firm_data_sub; /* internal sensor data, which should be
                                                  * sampled faster than the ssl vision data
                                                  */ 
        // ITPS::Subscriber<SSL_VisionData>  (trivial mode) .... To-do 


};

// alias
using MotionEKF = MotionEKF_Module;

/*  */

// Pseudo EKF
class VirtualMotionEKF : public MotionEKF_Module {
public:
    VirtualMotionEKF();
    ~VirtualMotionEKF();

    void task() override {}

    [[noreturn]] void task(ThreadPool& threadPool) override;

private:
    boost::shared_ptr<boost::asio::ip::udp::socket> socket;
    boost::shared_ptr<boost::asio::ip::udp::endpoint> grsim_endpoint;
    boost::shared_ptr<boost::asio::deadline_timer> timer;
    boost::asio::io_service io_service;

    arma::vec prev_disp = {0, 0};
    MotionEKF::BotData motion_data;


};