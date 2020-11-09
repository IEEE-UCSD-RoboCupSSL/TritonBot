#include "Main/settings.hpp"
#include <unistd.h>
#include "Utility/boost_logger.hpp"
#include <iostream>
#include <sstream>
#include <string>
#include "Utility/common.hpp"

static void help_print(B_Log& logger) {            
    std::stringstream ss;
    ss << "\nCommand: \n" 
       << "For Virtual Robots on the Simulator: \n" 
       << "\t./TritonBot.exe (-v) <port_base> (<vfirm_ip>) <vfirm_port> \n\n"
       << "\t\t-v: For controlling virtual robots in the simulator\n"
       << "\t\t<port_base>: specify the port base number to host the servers of THIS program on (port_base), (port_base+1), (port_base+2), and (port_base+3) \n"
       << "\t\t<vfirm_ip>: specify the ip address (in string) for the vfirm.exe program that virtualize robot's firmware layer. Default to LocalHost if not specified \n"
       << "\t\t<vfirm_port>: specify the port of the particular vfirm.exe program to connect\n"
       << "For Real Robots: \n"
       << "\t(WORK IN PROGRESS, NOT SUPPORTED FOR NOW)\n";
    logger.log(Info, ss.str());
}


bool process_args(int argc, char *argv[]) {
    B_Log logger;
    logger.add_tag("CMDArgument Processor");
    
    bool is_virtual = false;
    char option;
    while ( (option = getopt(argc, argv,":v")) != -1 ) {
        switch(option) {
            case 'v':
                is_virtual = true;
                break;
            case '?':
                B_Log err_logger;
                err_logger.add_tag("[setting.cpp]");
                err_logger.log(Error, std::string("Unknown option: ").append(1, (char)optopt));
                help_print(logger);
                std::exit(0);
                break;
        } 
    }

    if(is_virtual) {
        if ( optind + 3 == argc ) {
            // <port base>
            CONN_SERVER_PORT = std::stoi( std::string(argv[argc - 3]), nullptr, 10 );
            CMD_SERVER_PORT = CONN_SERVER_PORT + 1;
            GVISION_SERVER_PORT = CONN_SERVER_PORT + 2;
            EKF_SERVER_PORT = CONN_SERVER_PORT + 3;
            
            // <vfirm ip>
            VFIRM_IP_ADDR = std::string(argv[argc - 2]);
            
            // <vfirm port>
            VFIRM_IP_PORT = std::stoi( std::string(argv[argc - 1]), nullptr, 10 );
        }
        else if (optind + 2 == argc) {
            // <port base>
            CONN_SERVER_PORT = std::stoi( std::string(argv[argc - 2]), nullptr, 10 );
            CMD_SERVER_PORT = CONN_SERVER_PORT + 1;
            GVISION_SERVER_PORT = CONN_SERVER_PORT + 2;
            EKF_SERVER_PORT = CONN_SERVER_PORT + 3;
            
            // <vfirm port>
            VFIRM_IP_PORT = std::stoi( std::string(argv[argc - 1]), nullptr, 10 );
        }
        else {
            B_Log err_logger;
            err_logger.add_tag("[setting.cpp]");
            err_logger.log(Error, "Not enough arguments");
            help_print(logger);
            std::exit(0);
        }

        std::stringstream ss;
        ss << "\nThis program listens on LocalHost \n"
           << "\tConnection Server Port   : " + repr(CONN_SERVER_PORT) + "\n"
           << "\tCommand Server Port      : " + repr(CMD_SERVER_PORT) + "\n"
           << "\tGlobal Vision Server Port: " + repr(GVISION_SERVER_PORT) + "\n"
           << "\tEKF Server Port          : " + repr(EKF_SERVER_PORT) + "\n"
           << "Connecting to the Virtual Robot on grSim claimed by vfirm.exe listening on: \n"
           << "\t" + VFIRM_IP_ADDR + " " + repr(VFIRM_IP_PORT)  
           << std::endl;
        logger.log(Info, ss.str());
        return true;

    } 
    else {
        B_Log err_logger;
        err_logger.add_tag("[setting.cpp]");
        err_logger.log(Error, "Real Robot Mode temporarily not supported");
        help_print(logger);
        std::exit(0);
        return false;
    }
}