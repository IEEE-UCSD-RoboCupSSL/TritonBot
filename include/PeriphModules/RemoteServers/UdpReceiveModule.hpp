#pragma once
#include "Misc/PubSubSystem/Module.hpp"
#include "CoreModules/DataCmdTypes.hpp"
#include "Config/Config.hpp"

class UdpReceiveModule : public Module {
    public:
        // pass by copy on purpose to avoid multithreading synchronization concerns
        UdpReceiveModule(Config cfg) : config(cfg) {}
        virtual void task(ThreadPool& threadPool);
    protected:
        Config config;
};

