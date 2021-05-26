#pragma once

#include "Misc/PubSubSystem/Module.hpp"
#include "CoreModules/DataCmdTypes.hpp"
#include "Config/Config.hpp"



class BallCaptureModule : public Module {
    public:
        BallCaptureModule(Config& cfg) : config(cfg) {}
        virtual void task(ThreadPool& threadPool);
    private:
        Config config;
};