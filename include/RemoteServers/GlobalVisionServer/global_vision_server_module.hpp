#pragma once
#include "PubSubSystem/module.hpp"


class GlobalVisionServerModule : public Module {
    public:
        virtual void task() {}
        virtual void task(ThreadPool& thread_pool);

        virtual ~GlobalVisionServerModule() {}
};

using GlobalVisionServer = GlobalVisionServerModule;