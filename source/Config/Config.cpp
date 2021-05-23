#include <string>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <string>

#include "Misc/Utility/BoostLogger.hpp"
#include "Misc/Utility/Common.hpp"
#include "Config/Config.hpp"

unsigned int ROBOT_ID = 0;

unsigned int THREAD_POOL_SIZE = 30;

unsigned int INIT_DELAY = 1000; // 1 sec
unsigned int DEFAULT_SUBSCRIBER_TIMEOUT = 3000; // 3 sec

/* These values will be different for different robots, hence be reset in another file */
int TCP_PORT = 6000; // juts an example default val, will be reset in another code file
int UDP_PORT = 6001; // juts an example default val, will be reset in another code file


unsigned int SAFETY_EN_TIMEOUT = 500; // 500 ms

unsigned int CTRL_FREQUENCY = 500; // Hz

float NS_PID_AMP = 2.5; // for no-slowdown mode, pid const is multiplied by NS_PID_AMP

// Translational PID consts
float PID_TD_KP = 0.20;
float PID_TD_KI = 0.00;
float PID_TD_KD = 0.00;

// Rotational PID consts
float PID_RD_KP = 0.50;
float PID_RD_KI = 0.00;
float PID_RD_KD = 0.00;

// Correction magnitude to reduce the curve effect due to simutaneous translational and rotational motion 
float PID_TDRD_CORR = 1.0;
float PID_TVRD_CORR = 1.0;
float PID_TDRV_CORR = 1.0;
float PID_TVRV_CORR = 1.0;

// float PID_MAX_ROT_PERC = 30.00;


/*  */



