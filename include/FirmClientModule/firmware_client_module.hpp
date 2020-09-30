#pragma once
#include "PubSubSystem/module.hpp"


class FirmClientModule : public Module {
    public:
        virtual void task() = 0;
        virtual void task(ThreadPool& thread_pool) = 0;

        virtual ~FirmClientModule() {}
};