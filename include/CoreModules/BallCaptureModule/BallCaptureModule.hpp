#pragma once

#include "Misc/PubSubSystem/Module.hpp"
#include "CoreModules/DataCmdTypes.hpp"
#include "Config/Config.hpp"



class BallCaptureModule : public Module {
    public:

        // pass by copy on purpose to avoid multithreading synchronization concerns
        BallCaptureModule(Config cfg) : config(cfg) {}

    [[noreturn]] virtual void task(ThreadPool& threadPool);
    private:
        Config config;
};