#pragma once
#include "FirmClientModule/firmware_client_module.hpp"


class VFirmClient : public FirmClientModule {
    public:
        void task() {}
        void task(ThreadPool& thread_pool);
};