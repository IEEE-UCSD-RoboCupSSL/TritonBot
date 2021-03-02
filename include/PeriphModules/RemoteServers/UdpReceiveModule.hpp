#pragma once
#include "Misc/PubSubSystem/Module.hpp"
#include "Misc/PubSubSystem/Module.hpp"

class UdpReceiveModule : public Module {
    public:
        virtual void task() {}

    [[noreturn]] virtual void task(ThreadPool& thread_pool);

        virtual ~UdpReceiveModule() {}


};

using CMDServer = UdpReceiveModule;
