#pragma once

#include "Misc/PubSubSystem/Module.hpp"
#include "CoreModules/DataCmdTypes.hpp"
#include "DataFusion/BallDataFusion.hpp"
#include "DataFusion/BotDataFusion.hpp"




class DataProcessorModule : public Module {
    public:
        DataProcessorModule(BotDataFusion& botdf, BallDataFusion& balldf);
        virtual void task(ThreadPool& threadPool);
    private:
        BotDataFusion *botFusion;
        BallDataFusion *ballFusion;
};


