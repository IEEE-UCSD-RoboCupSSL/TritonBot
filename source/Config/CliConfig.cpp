#include "Config/CliConfig.hpp"
#include <string>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <string>

#include "Misc/Utility/BoostLogger.hpp"
#include "Misc/Utility/Common.hpp"

void help_print(BLogger& logger) {
    std::stringstream ss;
    ss << "\nCommand: \n"
       << "For Virtual Robots on the Simulator: \n"
       << "\t./TritonBot.exe (-v) (-t) (-d) (-c <path_to_config_file>) <port_base> \n"
       << "\t\t-v: for controlling virtual robots in the simulator\n"
       << "\t\t-t: test mode\n"
       << "\t\t-d: debug mode\n"
       << "\t\t-c <file path>: desinated a robot config file (.ini file)\n"
       << "\t\t<port_base>: specify the port base number to host the servers of THIS program on (port_base), (port_base+1) \n"
       << "For Real Robots: \n"
       << "\t(WORK IN PROGRESS, NOT SUPPORTED FOR NOW)\n";
    logger.log(Info, ss.str());
}


CliConfig processArgs(int argc, char *argv[]) {
    BLogger logger;
    logger.addTag("CLI config");

    CliConfig config;

    char option;
    while ( (option = getopt(argc, argv,"c:vtdh")) != -1 ) {
        switch(option) {
            case 'v': 
                config.isVirtual = true;
                logger(Info) << "[VirtualMode] enabled";
                break;
            case 't': 
                config.isTestMode = true;
                logger(Info) << "[TestMode] enabled";
                break;
            case 'd':
                BLogger::sink->set_filter(severity >= Debug);
                logger(Info) << "[DebugMode] enabled";
                break;
            /* case 'i':
                config.robotId = std::stoi(optarg);
                break; */
            case 'c':
                config.botConfigFilePath = std::string(optarg);
                logger.log(Info, "Config file desinated: " + config.botConfigFilePath);
                break;
            case 'h': {
                BLogger helpLogger;
                help_print(helpLogger);
                std::exit(0);
                break;
            }
            case '?': {
                BLogger errLogger;
                errLogger.addTag("[setting.cpp]");
                errLogger.log(Error, std::string("Unknown option: ").append(1, (char)optopt));
                help_print(errLogger);
                std::exit(0);
                break;
            }
        }
    }

    // logger.log(Info, "\033[0;32m Robot ID: " + repr(config.robotId) + "\033[0m");
    

    if(config.isVirtual) {
        if (optind == argc - 2) {
            config.simulatorName = std::string(argv[argc - 1]);

            // <port base>
            config.tcpPort = std::stoi(std::string(argv[argc - 2]));
            config.udpPort = config.tcpPort + 1;
        } else if (optind == argc - 1) {
            // <port base>
            config.tcpPort = std::stoi(std::string(argv[argc - 1]));
            config.udpPort = config.tcpPort + 1;
        } else if(optind == argc) {
            config.udpPort = config.tcpPort + 1;     
        } else {
            BLogger errLogger;
            errLogger.addTag("[Config.cpp]");
            errLogger.log(Error, "Too many arguments");
            help_print(logger);
            std::exit(0);
        }

        std::stringstream ss;
        ss << "\nThis program listens on LocalHost \n"
           << "\tTCP Port: " + repr(config.tcpPort) + "\n"
           << "\tUDP Port: " + repr(config.udpPort) << std::endl;
        logger.log(Info, ss.str());
    }
    else {
        BLogger errLogger;
        errLogger.addTag("[Config.cpp]");
        errLogger.log(Error, "Real Robot Mode temporarily not supported");
        help_print(logger);
        std::exit(0);
    }

    return config;
}