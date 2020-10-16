#pragma once
#include "PubSubSystem/module.hpp"
#include <armadillo>



class BallEKF_Module : public Module {
    public:

        struct BallData {
            arma::vec loc;
            arma::vec vel;
        };

        BallEKF_Module();
        virtual ~BallEKF_Module();

        virtual void task() = 0;
        virtual void task(ThreadPool& thread_pool) = 0;


    protected:
        virtual void init_subscribers();

        void publish_ball_data(BallData data);

    private:
        ITPS::NonBlockingPublisher<BallEKF_Module::BallData> ball_data_pub;
        // add ssl vision subscriber later

};

// alias
using BallEKF = BallEKF_Module;