#pragma once
#include "Misc/PubSubSystem/Module.hpp"
#include <armadillo>
#include "ProtoGenerated/RemoteAPI.pb.h"
#include "Misc/Utility/BoostLogger.hpp"
#include "CoreModules/DataCmdTypes.hpp"

class BallEKF_Module : public Module {
    public:
        BallEKF_Module();
        virtual ~BallEKF_Module();

        
        virtual void task(ThreadPool& threadPool) = 0;


    protected:
        virtual void init_subscribers();

        void publish_ball_data(BallData data);
        arma::vec get_ball_loc();
        arma::vec get_ball_vel();

        ITPS::FieldPublisher<BallData> ball_data_pub;
        ITPS::FieldSubscriber<arma::vec> ball_loc_sub; //("From:UdpReceiveModule", "BallPos(WorldFrame)"); 
        ITPS::FieldSubscriber<arma::vec> ball_vel_sub; //("From:UdpReceiveModule", "BallVel(WorldFrame)");
        
};

// alias
using BallEKF = BallEKF_Module;

// Pseudo EKF
class VirtualBallEKF : public BallEKF_Module {
public:
    VirtualBallEKF();
    ~VirtualBallEKF();

    void task() {}
    void task(ThreadPool& threadPool);

private:

    BallData ball_data;
};