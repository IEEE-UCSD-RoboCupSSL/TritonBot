#pragma once
#include <string>

extern unsigned int THREAD_POOL_SIZE;

extern std::string VFIRM_IP_ADDR;
extern unsigned int VFIRM_IP_PORT;


extern unsigned int VF_CMD_MQ_SIZE;
extern unsigned int VF_DATA_MQ_SIZE;

extern unsigned int VF_CMD_SUB_TIMEOUT;

extern unsigned int SAFETY_EN_TIMEOUT;

extern unsigned int CTRL_FREQUENCY;

extern float PID_TD_KP;
extern float PID_TD_KI;
extern float PID_TD_KD;

extern float PID_TV_KP;
extern float PID_TV_KI;
extern float PID_TV_KD;

extern float PID_RD_KP;
extern float PID_RD_KI;
extern float PID_RD_KD;

extern float PID_RV_KP;
extern float PID_RV_KI;
extern float PID_RV_KD;