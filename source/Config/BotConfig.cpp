#include "Config/BotConfig.hpp"
#include "Misc/Inih/inih/INIReader.h"



void processIni(std::string filepath, BotConfig& config) {
    
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
    logger(Info) << "file parsed";


    fclose(file);
}