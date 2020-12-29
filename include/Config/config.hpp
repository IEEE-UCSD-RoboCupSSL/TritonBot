#pragma once
#include <string>

/* Json Configurable Global Variables */
extern unsigned int THREAD_POOL_SIZE;

extern unsigned int INIT_DELAY;
extern unsigned int DEFAULT_SUBSCRIBER_TIMEOUT;

extern std::string VFIRM_IP_ADDR;
extern unsigned int VFIRM_IP_PORT;


extern unsigned int FIRM_CMD_MQ_SIZE;
extern unsigned int FIRM_DATA_MQ_SIZE;

extern unsigned int FIRM_CMD_SUB_TIMEOUT;

extern unsigned int SAFETY_EN_TIMEOUT;

extern unsigned int CTRL_FREQUENCY;


extern float NS_PID_AMP;

extern float PID_TD_KP;
extern float PID_TD_KI;
extern float PID_TD_KD;

// extern float PID_TV_KP;
// extern float PID_TV_KI;
// extern float PID_TV_KD;

extern float PID_RD_KP;
extern float PID_RD_KI;
extern float PID_RD_KD;

// extern float PID_RV_KP;
// extern float PID_RV_KI;
// extern float PID_RV_KD;

// extern float PID_DIR_KP;
// extern float PID_DIR_KI; 
// extern float PID_DIR_KD;

extern float PID_TDRD_CORR; // orientation correction scaling factor 
extern float PID_TVRD_CORR;
extern float PID_TDRV_CORR;
extern float PID_TVRV_CORR;

extern float PID_MAX_ROT_PERC; // maximum allowed rotational velocity in percentage of max vel


extern int CONN_SERVER_PORT;
extern int CMD_SERVER_PORT;
extern int GVISION_SERVER_PORT;
extern int EKF_SERVER_PORT;




extern int GRSIM_VISION_PORT;
extern std::string GRSIM_VISION_IP; 



/* Global Constants */
const std::size_t UDP_RBUF_SIZE = 1024;
const std::size_t UDP_WBUF_SIZE = 1024;



