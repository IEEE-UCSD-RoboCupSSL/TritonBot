#include <thread>
#include <chrono>
#include "Misc/Utility/Systime.hpp"

unsigned int millis(void) {
    auto t = std::chrono::high_resolution_clock::now();
    return (unsigned int)(double(t.time_since_epoch().count()) / 1000000.00f);
}

unsigned int micros(void) {
    auto t = std::chrono::high_resolution_clock::now();
    return (unsigned int)(double(t.time_since_epoch().count()) / 1000.00f);
}

void delay_us(unsigned int microseconds) {
    std::this_thread::sleep_for(std::chrono::microseconds(microseconds));
}

void delay(unsigned int milliseconds) {
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}



void delay(std::chrono::duration<float> period) {
    std::this_thread::sleep_for(period);
}