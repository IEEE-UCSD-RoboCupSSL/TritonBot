#pragma once
#include "Misc/PubSubSystem/Module.hpp"
#include "CoreModules/DataCmdTypes.hpp"
#include "Config/Config.hpp"


class TcpReceiveModule : public Module {
    public:
        TcpReceiveModule(Config& cfg) : config(cfg) {}
        virtual void task(ThreadPool& threadPool);
    protected:
        Config config;
};
