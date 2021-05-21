#pragma once
#include "Misc/PubSubSystem/Module.hpp"
#include "CoreModules/DataCmdTypes.hpp"
#include <armadillo>



class InternalVisionModule : public Module {
    public:


        InternalVisionModule();
        virtual ~InternalVisionModule();

        
        virtual void task(ThreadPool& threadPool) = 0;


    protected:
        virtual void init_subscribers();
        // void publish_ball_data(arma::vec location2d, arma::vec velocity2d);

    private:


};

using InternVision = InternalVisionModule;