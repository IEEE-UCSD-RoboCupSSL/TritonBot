#pragma once
#include <armadillo>
#include <string>
#include "CoreModules/DataCmdTypes.hpp"
#include "Misc/Utility/BoostLogger.hpp"


/** will separate this into a couple files in the future, I'm too lazy rn **/

class BotConfig {
public:


    BotConfig(bool __isVirtual) : isVirtual(__isVirtual) {}
protected:
    bool isVirtual;
};

class RealBotConfig : public BotConfig {
public:
    RealBotConfig() : BotConfig(false) {} 
};

class VirtualBotConfig : public BotConfig {
public:
    VirtualBotConfig() : BotConfig(true) {}
    virtual ~VirtualBotConfig() {}
    virtual bool isBallCloseEnoughToBot(BallData ballData, BotData botData) = 0;
    PIDConstants transDispConsts;
    PIDConstants rotatDispConsts;
};


class GrSimBotConfig : public VirtualBotConfig {

private:
    // unit: mm
    float const dribblerOffset = 105.0;
    float const ballNearBotZoneWidth = 500.0; 
    float const ballNearBotZoneHeight = 300.0;
    float const holdBallZoneWidth = 100.0;
    float const holdBallZoneHeight = 40.0;

public:
    GrSimBotConfig() : VirtualBotConfig() {}
    bool isBallCloseEnoughToBot(BallData ballData, BotData botData) {
        if(ballData.frame != botData.frame) {
            BLogger logger;
            logger.addTag("[BotConfig.hpp]");
            logger(Warning) << "ball data frame is inconsistant with bot data fram";
            return false;
        }
        arma::vec2 delta = ballData.pos - botData.pos;
        if (std::abs(delta(0)) < (ballNearBotZoneWidth / 2.0)
            && delta(1) < dribblerOffset + (ballNearBotZoneHeight / 2.0)
            && delta(1) > dribblerOffset - (ballNearBotZoneHeight / 2.0)) {
            return true;
        }
        return false;
    }
    bool isHoldingBall(BallData ballData, BotData botData) {
        if(ballData.frame != botData.frame) {
            BLogger logger;
            logger.addTag("[BotConfig.hpp]");
            logger(Warning) << "ball data frame is inconsistant with bot data fram";
            return false;
        }
        arma::vec2 delta = ballData.pos - botData.pos;

        // std::cout << delta << std::endl;

        if (std::abs(delta(0)) < (holdBallZoneWidth / 2.0)
            && delta(1) < dribblerOffset + (holdBallZoneHeight / 2.0)
            && delta(1) > dribblerOffset - (holdBallZoneHeight / 2.0)) {
            return true;
        }
        return false;
    }
};


class ErForceSimBotConfig : public VirtualBotConfig {
public:
    ErForceSimBotConfig() : VirtualBotConfig() {}
    bool isBallCloseEnoughToBot(BallData ballData, BotData botData) {
        // To-do
        return false;
    }
};



void processIni(std::string filepath, BotConfig& config);