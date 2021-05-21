#pragma once
#include "Misc/PubSubSystem/Module.hpp"
#include "Misc/PubSubSystem/Module.hpp"

class UdpReceiveModule : public Module {
    public:
        virtual void task(ThreadPool& threadPool);
};

