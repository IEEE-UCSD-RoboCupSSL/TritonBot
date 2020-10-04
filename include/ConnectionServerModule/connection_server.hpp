#pragma once
#include "ConnectionServerModule/connection_server_module.hpp"


class ConnectionServer : public ConnectionServerModule {
    public:
        void task() {}
        void task(ThreadPool& thread_pool);

};