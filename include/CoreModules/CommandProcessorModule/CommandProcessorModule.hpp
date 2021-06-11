#pragma once
#include "Config/Config.hpp"
#include "Misc/PubSubSystem/Module.hpp"

class CommandProcessorModule : public Module {
    public:
        // pass by copy on purpose to avoid multithreading synchronization concerns
        CommandProcessorModule(Config cfg) : config(cfg) {}
        virtual void task(ThreadPool& threadPool);
    protected:
        Config config;
};
