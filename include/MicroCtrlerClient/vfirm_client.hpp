#pragma once
#include "PubSubModule/module.hpp"


class VFirmClient : public Module {
    public:
        void task() {}
        void task(ThreadPool& thread_pool);
};