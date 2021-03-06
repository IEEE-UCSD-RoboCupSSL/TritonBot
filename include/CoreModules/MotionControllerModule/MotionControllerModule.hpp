#pragma once

#include "Misc/PubSubSystem/Module.hpp"
#include "CoreModules/DataCmdTypes.hpp"
#include "Config/Config.hpp"



class MotionControllerModule : public Module {
    public:
        // pass by copy on purpose to avoid multithreading synchronization concerns
        MotionControllerModule(Config cfg) : config(cfg) {}
        virtual void task(ThreadPool& threadPool);
    private:
        Config config;
};