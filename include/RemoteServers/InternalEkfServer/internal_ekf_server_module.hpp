#pragma once
#include "PubSubSystem/module.hpp"


class InternalEkfServerModule : public Module {
    public:
        virtual void task() {}
        virtual void task(ThreadPool& thread_pool);

        virtual ~InternalEkfServerModule() {}
};

using InternalEkfServer = InternalEkfServerModule;