#pragma once
#include "Misc/PubSubSystem/Module.hpp"
#include "CoreModules/DataCmdTypes.hpp"


class TcpReceiveModule : public Module {
    public:
        virtual void task(ThreadPool& threadPool);
};
