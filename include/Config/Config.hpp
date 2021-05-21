#pragma once
#include <string>

const std::size_t UDP_RBUF_SIZE = 1024;
const std::size_t UDP_WBUF_SIZE = 1024;

extern unsigned int ROBOT_ID;

extern unsigned int THREAD_POOL_SIZE;

extern unsigned int INIT_DELAY;
extern unsigned int DEFAULT_SUBSCRIBER_TIMEOUT;


extern int TCP_PORT;
extern int UDP_PORT;


extern unsigned int SAFETY_EN_TIMEOUT;
extern unsigned int CTRL_FREQUENCY;

extern float NS_PID_AMP;

extern float PID_TD_KP;
extern float PID_TD_KI;
extern float PID_TD_KD;


extern float PID_RD_KP;
extern float PID_RD_KI;
extern float PID_RD_KD;


extern float PID_TDRD_CORR; // orientation correction scaling factor 
extern float PID_TVRD_CORR;
extern float PID_TDRV_CORR;
extern float PID_TVRV_CORR;

// extern float PID_MAX_ROT_PERC; // maximum allowed rotational velocity in percentage of max vel

bool processArgs(int argc, char *argv[], bool& isTestMode);





