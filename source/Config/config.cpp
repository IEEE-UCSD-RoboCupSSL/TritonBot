#include "Config/config.hpp"
#include <string>

unsigned int THREAD_POOL_SIZE = 20;

unsigned int INIT_DELAY = 1000; // 1 sec
unsigned int DEFAULT_SUBSCRIBER_TIMEOUT = 3000; // 3 sec

std::string VFIRM_IP_ADDR = "127.0.0.1";
unsigned int VFIRM_IP_PORT = 8888;


unsigned int FIRM_CMD_MQ_SIZE = 10;
unsigned int FIRM_DATA_MQ_SIZE = 10;

unsigned int FIRM_CMD_SUB_TIMEOUT = 100; // 100 ms


unsigned int SAFETY_EN_TIMEOUT = 500; // 500 ms



unsigned int CTRL_FREQUENCY = 500; // Hz



float PID_TD_KP = 0.08;
float PID_TD_KI = 0.00;
float PID_TD_KD = 0.00;

// float PID_TV_KP = 1.00;
// float PID_TV_KI = 0.00;
// float PID_TV_KD = 0.00;

float PID_RD_KP = 0.60;
float PID_RD_KI = 0.00;
float PID_RD_KD = 0.00;

// float PID_RV_KP = 1.00;
// float PID_RV_KI = 0.00;
// float PID_RV_KD = 0.00;


float PID_DIR_KP = 1.00;
float PID_DIR_KI = 0.00; 
float PID_DIR_KD = 0.00;