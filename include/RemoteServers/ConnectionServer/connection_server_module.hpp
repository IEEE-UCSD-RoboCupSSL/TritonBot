#pragma once
#include "PubSubSystem/module.hpp"


class ConnectionServerModule : public Module {
    public:
        virtual void task() {}
        virtual void task(ThreadPool& thread_pool);

        virtual ~ConnectionServerModule() {}
};

using ConnectionServer = ConnectionServerModule;