#pragma once
#include "Misc/PubSubSystem/Module.hpp"
#include "CoreModules/DataCmdTypes.hpp"

class UdpReceiveModule : public Module {
    public:
        virtual void task(ThreadPool& threadPool);
};

