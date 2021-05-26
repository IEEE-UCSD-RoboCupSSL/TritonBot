#pragma once
#include "Misc/PubSubSystem/Module.hpp"
#include "CoreModules/DataCmdTypes.hpp"
#include "Config/Config.hpp"



class McuClientModule : public Module {
    public:
        McuClientModule(Config& cfg) : config(cfg) {}
        virtual void task(ThreadPool& threadPool);
    protected:
        Config config;
};

