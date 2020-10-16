#pragma once
#include "PubSubSystem/module.hpp"
#include <armadillo>



class InternalVisionModule : public Module {
    public:


        InternalVisionModule();
        virtual ~InternalVisionModule();

        virtual void task() = 0;
        virtual void task(ThreadPool& thread_pool) = 0;


    protected:
        virtual void init_subscribers();
        // void publish_ball_data(arma::vec location2d, arma::vec velocity2d);

    private:


};

using InternVision = InternalVisionModule;