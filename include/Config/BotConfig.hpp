#pragma once
#include <armadillo>
#include <string>
#include "CoreModules/DataCmdTypes.hpp"
#include "Misc/Utility/BoostLogger.hpp"
#include "Misc/Utility/ClockUtil.hpp"
#include "Config/ModuleFrequencies.hpp"


/** will separate this into a couple files in the future, I'm too lazy rn **/

class BotConfig {
public:
    unsigned int pidControlFrequency;
    PIDConstants transDispConsts;
    PIDConstants rotatDispConsts;
    float noSlowDownPidAmp;
    float pidTdrdCorr;
    float pidTdrvCorr;
    float pidTvrdCorr;
    float pidTvrvCorr;

    BotConfig(bool __isVirtual) : isVirtual(__isVirtual) { type = "BotConfig";}
    inline std::string getType() {return type;}
protected:  
    bool isVirtual;
    std::string type;
};

class RealBotConfig : public BotConfig {
public:
    RealBotConfig() : BotConfig(false) {type = "RealBotConfig";} 
};

class VirtualBotConfig : public BotConfig {
public:
    VirtualBotConfig() : BotConfig(true) {type = "VirtualBotConfig";}
    virtual ~VirtualBotConfig() {}
    virtual bool isBallCloseEnoughToBot(BallData ballData, BotData botData) = 0;
};


class GrSimBotConfig : public VirtualBotConfig {

private:
    // unit: mm
    float const dribblerOffset = 105.0;
    float const ballNearBotZoneWidth = 500.0; 
    float const ballNearBotZoneHeight = 300.0;
    float const holdBallZoneWidth = 100.0;
    float const holdBallZoneHeight = 40.0;

    std::chrono::steady_clock::time_point t0; 
    long samplingFrequency = 10; // Hz
    long hbCnt = 0, roundTotal = 0; 
    float percentThreshHold = 0.6; // if hbCnt/roundTotal >= 60 %, ball is dribbled 
    bool hbResult = false;

public:
    GrSimBotConfig() : VirtualBotConfig() {
        type = "GrSimBotConfig";
        t0 = CHRONO_NOW;
    }
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
            hbCnt++;
        }
        roundTotal++;

        if(CHRONO_NOW - t0 > TO_PERIOD(samplingFrequency)) {
            if((float)hbCnt / (float)roundTotal >= percentThreshHold) {
                hbResult = true;
            } else {
                hbResult = false;
            }
            t0 = CHRONO_NOW;
            hbCnt = 0;
            roundTotal = 0;
        }
        return hbResult;
    }
};


class ErForceSimBotConfig : public VirtualBotConfig {
public:
    ErForceSimBotConfig() : VirtualBotConfig() {type = "ErForceSimBotConfig";}
    bool isBallCloseEnoughToBot(BallData ballData, BotData botData) {
        // To-do
        return false;
    }
};



void processIni(std::string filepath, std::shared_ptr<BotConfig> config);