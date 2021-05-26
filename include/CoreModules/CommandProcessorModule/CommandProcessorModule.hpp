#pragma once
#include "Config/Config.hpp"
#include "Misc/PubSubSystem/Module.hpp"

class CommandProcessorModule : public Module {
    public:
        CommandProcessorModule(Config& cfg) : config(cfg) {}
        virtual void task(ThreadPool& threadPool);
    protected:
        Config config;
};
