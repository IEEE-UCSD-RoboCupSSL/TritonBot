#pragma once
#include "Config/BotConfig.hpp"
#include "Misc/PubSubSystem/PubSub.hpp"

struct CliConfig {
    /** assigned with a defaults, subject to change **/
    bool isVirtual = true;
    bool isTestMode = false;
    //unsigned int robotId = 0;
    unsigned int tcpPort = 6000;
    unsigned int udpPort = 6001;
    unsigned int mcuTopTcpPort = 6002;
    unsigned int mcuTopUdpReadPort = 6003;
    unsigned int mcuTopUdpWritePort = 6004;
    std::string simulatorName = "grSim";
    std::string botConfigFilePath = "";
    std::string liveMonitorTarget = ""; 
};

CliConfig processArgs(int argc, char *argv[]);