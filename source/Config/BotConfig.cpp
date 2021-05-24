#include "Config/BotConfig.hpp"
#include "Misc/Inih/inih/INIReader.h"
#include "Misc/Utility/Common.hpp"



void processIni(std::string filepath, std::shared_ptr<BotConfig> config) {
    
    FILE* file = fopen(filepath.c_str(), "r");
    if(!file) {
        BLogger errLogger;
        errLogger.log(Error, "can't open file: " + filepath );
        fclose(file);
        std::exit(0);
    }
    INIReader reader(file);
    if(reader.ParseError() != 0) {
        BLogger errLogger;
        errLogger.log(Error, "can't parse file: " + filepath );
        fclose(file);
        std::exit(0);
    }
    BLogger logger;
    logger.addTag("[Bot config]");
    logger(Info) << "file parsed";

    std::string botType = reader.Get("robot type", "bot_type", "Unknown");
    if(botType == "grsim_vbots") {
        // std::cout << "....." << std::endl;
        std::shared_ptr<GrSimBotConfig> cfg = std::static_pointer_cast<GrSimBotConfig>(config);
        cfg->transDispConsts.Kp = reader.GetFloat("pid controller constants", 
                        "pid_td_kp", 0.0);
        cfg->transDispConsts.Ki = reader.GetFloat("pid controller constants", 
                        "pid_td_ki", 0.0);
        cfg->transDispConsts.Kd = reader.GetFloat("pid controller constants", 
                        "pid_td_kd", 0.0);
        cfg->rotatDispConsts.Kp = reader.GetFloat("pid controller constants", 
                        "pid_rd_kp", 0.0);
        cfg->rotatDispConsts.Ki = reader.GetFloat("pid controller constants", 
                        "pid_rd_ki", 0.0);
        cfg->rotatDispConsts.Kd = reader.GetFloat("pid controller constants", 
                        "pid_rd_kd", 0.0);
        cfg->noSlowDownPidAmp = reader.GetFloat("pid controller constants", 
                        "ns_pid_amp", 0.0);

        logger.log(Info, "Parsed from " + filepath + ":");
        logger.log(Info, "\t\tNoSlowDownAmplifierConst: " + repr(cfg->noSlowDownPidAmp));
        logger.log(Info, "\t\tTransPid: <" + repr(cfg->transDispConsts.Kp) + "," 
                                      + repr(cfg->transDispConsts.Ki) + ","
                                      + repr(cfg->transDispConsts.Kd) + ">");
        logger.log(Info, "\t\tRotatPid: <" + repr(cfg->rotatDispConsts.Kp) + "," 
                                      + repr(cfg->rotatDispConsts.Ki) + ","
                                      + repr(cfg->rotatDispConsts.Kd) + ">");

    }


    fclose(file);
}