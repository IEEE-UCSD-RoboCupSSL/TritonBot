#pragma once
#include "Config/BotConfig.hpp"

struct CliConfig {
    bool isVirtual = true;
    bool isTestMode = false;
    //unsigned int robotId = 0;
    unsigned int tcpPort = 6000;
    unsigned int udpPort = 6001;
    std::string simulatorName = "grSim";
};

CliConfig processArgs(int argc, char *argv[]);