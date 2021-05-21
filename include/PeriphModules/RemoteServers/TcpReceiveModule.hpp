#pragma once
#include "Misc/PubSubSystem/Module.hpp"


class TcpReceiveModule : public Module {
    public:
        virtual void task(ThreadPool& threadPool);
};
