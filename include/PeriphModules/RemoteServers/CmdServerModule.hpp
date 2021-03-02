#pragma once
#include "Misc/PubSubSystem/Module.hpp"


class CMDServerModule : public Module {
    public:
        virtual void task() {}

    [[noreturn]] virtual void task(ThreadPool& thread_pool);

        virtual ~CMDServerModule() {}


};

using CMDServer = CMDServerModule;
