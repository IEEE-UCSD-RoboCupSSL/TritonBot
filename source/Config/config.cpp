#include "Config/config.hpp"
#include <string>

unsigned int THREAD_POOL_SIZE = 30;

unsigned int INIT_DELAY = 1000; // 1 sec
unsigned int DEFAULT_SUBSCRIBER_TIMEOUT = 3000; // 3 sec

std::string VFIRM_IP_ADDR = "127.0.0.1";
unsigned int VFIRM_IP_PORT = 8888;


unsigned int FIRM_CMD_MQ_SIZE = 1;
unsigned int FIRM_DATA_MQ_SIZE = 10;

unsigned int FIRM_CMD_SUB_TIMEOUT = 100; // 100 ms


unsigned int SAFETY_EN_TIMEOUT = 500; // 500 ms



unsigned int CTRL_FREQUENCY = 500; // Hz



float PID_TD_KP = 0.06;
float PID_TD_KI = 0.00;
float PID_TD_KD = 0.00;

// float PID_TV_KP = 1.00;
// float PID_TV_KI = 0.00;
// float PID_TV_KD = 0.00;

float PID_RD_KP = 0.30;
float PID_RD_KI = 0.00;
float PID_RD_KD = 0.00;

// float PID_RV_KP = 1.00;
// float PID_RV_KI = 0.00;
// float PID_RV_KD = 0.00;

float PID_TDRD_CORR = 1.0;
float PID_TVRD_CORR = 1.0;
float PID_TDRV_CORR = 1.0;
float PID_TVRV_CORR = 1.0;

float PID_MAX_ROT_PERC = 30.00;



int CONN_SERVER_PORT = 6000;
int CMD_SERVER_PORT = 6001;
int GVISION_SERVER_PORT = 6002;
int EKF_SERVER_PORT = 6003;



int GRSIM_VISION_PORT = 10020;
std::string GRSIM_VISION_IP = "224.5.23.2";