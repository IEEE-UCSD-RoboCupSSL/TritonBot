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

};


class GrSimBotConfig : public VirtualBotConfig {

private:
    // unit: mm
    float const dribblerOffset = 105.0;
    float const ballNearBotZoneWidth = 250.0; 
    float const ballNearBotZoneHeight = 200.0;
    float const holdBallZoneWidth = 80.0;
    float const holdBallZoneHeight = 20.0;

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
        if (std::abs(delta(0)) < ballNearBotZoneWidth
            && delta(1) < dribblerOffset + ballNearBotZoneHeight
            && delta(1) > dribblerOffset - ballNearBotZoneHeight) {
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
        if (std::abs(delta(0)) < holdBallZoneWidth
            && delta(1) < dribblerOffset + holdBallZoneHeight
            && delta(1) > dribblerOffset - holdBallZoneHeight) {
            return true;
        }
        return false;
    }
};


class ErForceSimBotConfig : public VirtualBotConfig {
public:
    ErForceSimBotConfig() : VirtualBotConfig() {}

};