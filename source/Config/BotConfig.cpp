#include "Config/BotConfig.hpp"
#include "Misc/Inih/inih/INIReader.h"
#include "Misc/Utility/Common.hpp"
#include <cassert>



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
        //std::cout << config->getType() << std::endl;
        assert(config->getType() == "GrSimBotConfig");
        std::shared_ptr<GrSimBotConfig> cfg = std::static_pointer_cast<GrSimBotConfig>(config);
    }

    config->pidControlFrequency = reader.GetInteger("pid controller constants", 
                    "frequency", 500);

    config->posPidConsts.Kp = reader.GetFloat("pid controller constants", 
                    "pid_td_kp", 0.2);
    config->posPidConsts.Ki = reader.GetFloat("pid controller constants", 
                    "pid_td_ki", 0.0);
    config->posPidConsts.Kd = reader.GetFloat("pid controller constants", 
                    "pid_td_kd", 0.0);

    config->anglePidConsts.Kp = reader.GetFloat("pid controller constants", 
                    "pid_rd_kp", 0.5);
    config->anglePidConsts.Ki = reader.GetFloat("pid controller constants", 
                    "pid_rd_ki", 0.0);
    config->anglePidConsts.Kd = reader.GetFloat("pid controller constants", 
                    "pid_rd_kd", 0.0);

    config->noSlowDownPidAmp = reader.GetFloat("pid controller constants", 
                    "ns_pid_amp", 2.5);

    config->pidTdrdCorr = reader.GetFloat("pid controller constants", 
                    "pid_tdrd_corr", 1.0);
    config->pidTdrvCorr = reader.GetFloat("pid controller constants", 
                    "pid_tdrv_corr", 1.0);
    config->pidTvrdCorr = reader.GetFloat("pid controller constants", 
                    "pid_tvrd_corr", 1.0);
    config->pidTvrvCorr = reader.GetFloat("pid controller constants", 
                    "pid_tvrv_corr", 1.0);



    config->dribblerOffset = reader.GetFloat("dribbler setting", "dribblerOffset", 85.0);
    config->ballNearBotZoneHeight = reader.GetFloat("dribbler setting", "ballNearBotZoneHeight", 500.0);
    config->ballNearBotZoneWidth = reader.GetFloat("dribbler setting", "ballNearBotZoneWidth", 300.0);
    config->holdBallZoneHeight = reader.GetFloat("dribbler setting", "holdBallZoneHeight", 64.0);
    config->holdBallZoneWidth = reader.GetFloat("dribbler setting", "holdBallZoneWidth", 85.0);
    config->holdBallPercentThreshHold = reader.GetFloat("dribbler setting", "holdBallPercentThreshHold", 0.6);
    config->botStopBeforeBallDistance = reader.GetFloat("dribbler setting", "botStopBeforeBallDistance", 500);


    logger.log(Info, "Parsed from " + filepath + ":");
    logger.log(Info, "\t\tPidControlFrequency: " + repr(config->pidControlFrequency));
    logger.log(Info, "\t\tNoSlowDownAmplifierConst: " + repr(config->noSlowDownPidAmp));
    logger.log(Info, "\t\tTransPid: <" + repr(config->posPidConsts.Kp) + "," 
                                    + repr(config->posPidConsts.Ki) + ","
                                    + repr(config->posPidConsts.Kd) + ">");
    logger.log(Info, "\t\tRotatPid: <" + repr(config->anglePidConsts.Kp) + "," 
                                    + repr(config->anglePidConsts.Ki) + ","
                                    + repr(config->anglePidConsts.Kd) + ">");

    logger.log(Info, "\t\tpidTdrdCorr: " + repr(config->pidTdrdCorr));
    logger.log(Info, "\t\tpidTdrvCorr: " + repr(config->pidTdrvCorr));
    logger.log(Info, "\t\tpidTvrdCorr: " + repr(config->pidTvrdCorr));
    logger.log(Info, "\t\tpidTvrvCorr: " + repr(config->pidTvrvCorr));

    fclose(file);
}