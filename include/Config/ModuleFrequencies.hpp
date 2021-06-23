#pragma once


//#define TO_PERIOD(freq) std::chrono::nanoseconds(int(1e9/freq))
#define TO_PERIOD(freq) std::chrono::milliseconds((int)(1000.00f / (double)freq))

/* Module Frequencies */
#define TCP_RECEIVE_FREQUENCY 100 //Hz 
#define UDP_RECEIVE_FREQUENCY 500 //Hz
#define COMMAND_PROCESSOR_FREQUENCY 500 //Hz
#define DATA_PROCESSOR_FREQUENCY 500 //Hz
// MOTION_CONTROLLER_FREQUENCY will be set in a .ini file 
#define BALL_CAPTURE_FREQUENCY 500 //Hz
#define MCU_CLIENT_FREQUENCY 500 //Hz