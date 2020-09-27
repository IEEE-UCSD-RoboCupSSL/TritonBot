#pragma once
#include "PubSubModule/module.hpp"


class MicroCtrlerClientModule : public Module {
    public:
        virtual void task() = 0;
        virtual void task(ThreadPool& thread_pool) = 0;

        virtual ~MicroCtrlerClientModule() {}
};