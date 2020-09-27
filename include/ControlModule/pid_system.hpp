#pragma once

#include "ControlModule/control_module.hpp"


class PID_System : public ControlModule {
    public: 
        PID_System();

        void task() {}
        void task(ThreadPool& thread_pool);

};