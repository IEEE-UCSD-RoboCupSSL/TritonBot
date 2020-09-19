#include <iostream>
#include "PubSubModule/thread_pool.hpp"
#include "MicroCtrlerInterface/vfirm_client.hpp"
#include "Utility/boost_logger.hpp"

int main(int arc, char *argv[]) {

    B_Log::static_init();
    B_Log::sink->set_filter(severity >= Info);

/* thread pool version */
    ThreadPool thread_pool(20); // pre-allocate 10 threads in a pool

    VFirmClient vfirm_client_module;

    vfirm_client_module.run(thread_pool);

    while(1);

    return 0;
}