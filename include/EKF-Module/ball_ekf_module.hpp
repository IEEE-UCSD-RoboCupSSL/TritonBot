#pragma once
#include "PubSubSystem/module.hpp"
#include <armadillo>
#include "ProtoGenerated/RemoteAPI.pb.h"
#include "EKF-Module/motion_ekf_module.hpp"



class BallEKF_Module : public Module {
    public:

        struct BallData {
            arma::vec disp;
            arma::vec vel;
        };

        BallEKF_Module();
        virtual ~BallEKF_Module();

        
        virtual void task(ThreadPool& thread_pool) = 0;


    protected:
        virtual void init_subscribers();

        void publish_ball_data(BallData data);

        ITPS::NonBlockingPublisher<BallEKF_Module::BallData> ball_data_pub;
        // add ssl vision subscriber later
        
};

// alias
using BallEKF = BallEKF_Module;