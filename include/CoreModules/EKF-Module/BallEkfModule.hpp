#pragma once
#include "Misc/PubSubSystem/Module.hpp"
#include <armadillo>
#include "ProtoGenerated/RemoteAPI.pb.h"
#include "MotionEkfModule.hpp"
#include "BallEkfModule.hpp"
#include <armadillo>
#include "Misc/Utility/BoostLogger.hpp"
#include "MotionEkfModule.hpp"
#include <armadillo>

class BallEKF_Module : public Module {
    public:

        /* Ball frame: Blue team gate is at about (0, -4500) while yellow team gate is at about (0, 4500)
        When facing yellow team gate, the eastward direction is the positive direction while westward direction
        is the negative direction. */
        struct BallData {
            arma::vec disp; // Location of ball relative to center of the field in ball frame.
            arma::vec vel;  // Velocity of ball in ball frame
        };

        BallEKF_Module();
        virtual ~BallEKF_Module();

        
        virtual void task(ThreadPool& threadPool) = 0;


    protected:
        virtual void init_subscribers();

        void publish_ball_data(BallData data);
        arma::vec get_ball_loc();
        arma::vec get_ball_vel();

        ITPS::FieldPublisher<BallEKF_Module::BallData> ball_data_pub;
        ITPS::FieldSubscriber<arma::vec> ball_loc_sub; //("GVision Server", "BallPos(BodyFrame)"); 
        ITPS::FieldSubscriber<arma::vec> ball_vel_sub; //("GVision Server", "BallVel(BodyFrame)");
        
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

    BallEKF::BallData ball_data;
};