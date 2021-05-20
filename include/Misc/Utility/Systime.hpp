#ifndef __SYSTIME_H
#define __SYSTIME_H
#include <chrono>

#define DURATION_MILLIS_COUNT(time_point) std::chrono::duration_cast<std::chrono::milliseconds> (time_point).count()
#define CHRONO_NOW std::chrono::steady_clock::now()


unsigned int millis(void);

unsigned int micros(void);

void delay_us(unsigned int microseconds);

void delay(unsigned int milliseconds);

void delay(std::chrono::duration<float> period);


#endif 