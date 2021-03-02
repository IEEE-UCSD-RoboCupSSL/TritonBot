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

std::string VFIRM_IP_ADDR = "127.0.0.1"; // juts an example default val, will be reset in another code file
unsigned int VFIRM_IP_PORT = 8888; // juts an example default val, will be reset in another code file

int GRSIM_VISION_PORT = 10020; // juts an example default val, will be reset in another code file
std::string GRSIM_VISION_IP = "224.5.23.2"; // juts an example default val, will be reset in another code file

/* These values will be different for different robots, hence be reset in another file */
int CONN_SERVER_PORT = 6000; // juts an example default val, will be reset in another code file
int CMD_SERVER_PORT = 6001; // juts an example default val, will be reset in another code file
int EKF_SERVER_PORT = 6002; // juts an example default val, will be reset in another code file
int GVISION_SERVER_PORT = 6003; // juts an example default val, will be reset in another code file




unsigned int FIRM_CMD_MQ_SIZE = 1;
unsigned int FIRM_DATA_MQ_SIZE = 10;

unsigned int FIRM_CMD_SUB_TIMEOUT = 100; // 100 ms


unsigned int SAFETY_EN_TIMEOUT = 500; // 500 ms



unsigned int CTRL_FREQUENCY = 500; // Hz


float NS_PID_AMP = 2.5; // for no-slowdown mode, pid const is multiplied by NS_PID_AMP

// Translational PID consts
float PID_TD_KP = 0.20;
float PID_TD_KI = 0.00;
float PID_TD_KD = 0.00;

// Rotational PID consts
float PID_RD_KP = 0.50;
float PID_RD_KI = 0.00;
float PID_RD_KD = 0.00;

// Correction magnitude to reduce the curve effect due to simutaneous translational and rotational motion 
float PID_TDRD_CORR = 1.0;
float PID_TVRD_CORR = 1.0;
float PID_TDRV_CORR = 1.0;
float PID_TVRV_CORR = 1.0;

// float PID_MAX_ROT_PERC = 30.00;


/*  */


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
            EKF_SERVER_PORT = CONN_SERVER_PORT + 2;
            GVISION_SERVER_PORT = CONN_SERVER_PORT + 3;

            // <vfirm ip>
            VFIRM_IP_ADDR = std::string(argv[argc - 2]);

            // <vfirm port>
            VFIRM_IP_PORT = std::stoi( std::string(argv[argc - 1]), nullptr, 10 );
        }
        else if (optind + 2 == argc) {
            // <port base>
            CONN_SERVER_PORT = std::stoi( std::string(argv[argc - 2]), nullptr, 10 );
            CMD_SERVER_PORT = CONN_SERVER_PORT + 1;
            EKF_SERVER_PORT = CONN_SERVER_PORT + 2;
            GVISION_SERVER_PORT = CONN_SERVER_PORT + 3;

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
