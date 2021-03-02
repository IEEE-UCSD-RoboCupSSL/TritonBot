#pragma once
#include "Misc/PubSubSystem/Module.hpp"


class ConServerModule : public Module {
    public:
        virtual void task() {}
        virtual void task(ThreadPool& thread_pool);

        virtual ~ConServerModule() {}
};

using ConnectionServer = ConServerModule;