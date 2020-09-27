#include "Config/config.hpp"
#include <string>

unsigned int THREAD_POOL_SIZE = 20;

std::string VFIRM_IP_ADDR = "127.0.0.1";
unsigned int VFIRM_IP_PORT = 8888;


unsigned int VF_CMD_MQ_SIZE = 10;
unsigned int VF_DATA_MQ_SIZE = 10;

unsigned int VF_CMD_SUB_TIMEOUT = 100; // 100 ms


unsigned int SAFETY_EN_TIMEOUT = 500; // 500 ms