#pragma once


#define TO_PERIOD(freq) std::chrono::nanoseconds(int(1e9/freq))


/* Module Frequencies */
#define TCP_RECEIVE_FREQUENCY 10 //Hz 
#define UDP_RECEIVE_FREQUENCY 500 //Hz
