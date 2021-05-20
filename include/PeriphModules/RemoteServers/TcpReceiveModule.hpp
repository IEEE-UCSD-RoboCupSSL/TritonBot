#pragma once
#include "Misc/PubSubSystem/Module.hpp"


class TcpReceiveModule : public Module {
    public:
        virtual void task() {}
        virtual void task(ThreadPool& threadPool);

        virtual ~TcpReceiveModule() {}
};

using ConnectionServer = TcpReceiveModule;