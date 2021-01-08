#pragma once
#include "PubSubSystem/module.hpp"


class CMDServerModule : public Module {
    public:
        virtual void task() {}

    [[noreturn]] virtual void task(ThreadPool& thread_pool);

        virtual ~CMDServerModule() {}


};

using CMDServer = CMDServerModule;
