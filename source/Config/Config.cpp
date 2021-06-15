#include <string>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <string>

#include "Misc/Utility/BoostLogger.hpp"
#include "Misc/Utility/Common.hpp"
#include "Config/Config.hpp"


unsigned int THREAD_POOL_SIZE = 30;
unsigned int INIT_DELAY = 1000; // 1 sec
unsigned int DEFAULT_SUBSCRIBER_TIMEOUT = 3000; // 3 sec
unsigned int SAFETY_EN_TIMEOUT = 500; // 500 ms


