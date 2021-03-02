#pragma once
#include "Misc/PubSubSystem/Module.hpp"


class GVisServerModule : public Module {
    public:
        virtual void task() {}
        virtual void task(ThreadPool& thread_pool);

        virtual ~GVisServerModule() {}
};

using GlobalVisionServer = GVisServerModule;