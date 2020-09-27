#pragma once
#include "MicroCtrlerClient/microctrler_client_module.hpp"


class VFirmClient : public MicroCtrlerClientModule {
    public:
        void task() {}
        void task(ThreadPool& thread_pool);
};