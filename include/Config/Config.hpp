#pragma once
#include <string>
#include "Config/BotConfig.hpp"
#include "Config/CliConfig.hpp"

struct Config {
    Config(CliConfig& clicfg, std::shared_ptr<BotConfig> botcfg) 
        : cliConfig(clicfg), botConfig(botcfg) {}
    CliConfig cliConfig;
    std::shared_ptr<BotConfig> botConfig;
};



const std::size_t UDP_RBUF_SIZE = 1024;
const std::size_t UDP_WBUF_SIZE = 1024;


extern unsigned int THREAD_POOL_SIZE;

extern unsigned int INIT_DELAY;
extern unsigned int DEFAULT_SUBSCRIBER_TIMEOUT;



extern unsigned int SAFETY_EN_TIMEOUT;
