#pragma once
#include "Config/BotConfig.hpp"

struct CliConfig {
    bool isVirtual = true;
    bool isTestMode = false;
    //unsigned int robotId = 0;
    unsigned int tcpPort = 6000;
    unsigned int udpPort = 6001;
    unsigned int mcuTopTcpPort = 6002;
    std::string simulatorName = "grSim";
    std::string botConfigFilePath = "";
};

CliConfig processArgs(int argc, char *argv[]);