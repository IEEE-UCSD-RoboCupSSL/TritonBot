#pragma once
#include "Misc/PubSubSystem/Module.hpp"


class FirmClientModule : public Module {
    public:
        
        virtual void task(ThreadPool& threadPool) = 0;

        virtual ~FirmClientModule() {}
};

class VFirmClient : public FirmClientModule {
public:
    void task() {}
    void task(ThreadPool& threadPool);
};