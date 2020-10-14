#pragma once
#include "PubSubSystem/module.hpp"


class CommandServerModule : public Module {
    public:
        virtual void task() {}
        virtual void task(ThreadPool& thread_pool);

        virtual ~CommandServerModule() {}
};

using CMDServerModule = CommandServerModule;
using CMDServer = CMDServerModule;