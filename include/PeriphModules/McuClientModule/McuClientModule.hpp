#pragma once
#include "Misc/PubSubSystem/Module.hpp"
#include "CoreModules/DataCmdTypes.hpp"
#include "Config/Config.hpp"



class McuClientModule : public Module {
    public:
        // pass by copy on purpose to avoid multithreading synchronization concerns
        McuClientModule(Config cfg) : config(cfg) {
            isBigEndian = config.isBigEndian;
        }
        virtual void task(ThreadPool& threadPool);
        static bool isBigEndian;
    protected:
        Config config;
};


class McuClientModuleMonitor : public ModuleMonitor {
    public:
        // pass by copy on purpose to avoid multithreading synchronization concerns
        McuClientModuleMonitor(int samplingPeriodMs) : samplingPeriod(samplingPeriodMs) {
            ModuleMonitor::moduleMonitorMap["McuClientModule"] = this;

        }
        virtual void task(ThreadPool& threadPool);
    protected:
        int samplingPeriod;
};

