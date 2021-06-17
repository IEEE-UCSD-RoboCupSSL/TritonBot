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
    PIDConstants posPidConsts;
    PIDConstants anglePidConsts;
    float noSlowDownPidAmp;
    float pidTdrdCorr;
    float pidTdrvCorr;
    float pidTvrdCorr;
    float pidTvrvCorr;

    BotConfig(bool __isVirtual) : isVirtual(__isVirtual) { type = "BotConfig"; }

    inline std::string getType() { return type; }

    // c++ note: return by copy will be optimized by most c++ compiler via RVO (return value optimization)
    // c++ note2: const reference is used because it prolongs the life time of rvalue parameter (rval: temporary value) (plz google "const reference rvalue")
    virtual MotionCommand
    autoBallCaptureSolution(bool isHoldingBall, const BallData &ballData, const BotData &botData,
                            double const interpolationRate, double angle) = 0;

protected:
    bool isVirtual;
    std::string type;
};

class RealBotConfig : public BotConfig {
public:
    RealBotConfig() : BotConfig(false) { type = "RealBotConfig"; }

    virtual MotionCommand
    autoBallCaptureSolution(bool isHoldingBall, const BallData &ballData, const BotData &botData,
                            double const interpolationRate, double angle) = 0;
};

class VirtualBotConfig : public BotConfig {
public:
    VirtualBotConfig() : BotConfig(true) { type = "VirtualBotConfig"; }

    virtual ~VirtualBotConfig() {}

    virtual bool isBallCloseEnoughToBot(const BallData &ballData, const BotData &botData) = 0;

    virtual MotionCommand
    autoBallCaptureSolution(bool isHoldingBall, const BallData &ballData, const BotData &botData,
                            double const interpolationRate, double angle) = 0;
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

    bool isBallCloseEnoughToBot(const BallData &ballData, const BotData &botData) override {
        if (ballData.frame != botData.frame) {
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

    bool isHoldingBall(const BallData &ballData, const BotData &botData) {
        if (ballData.frame != botData.frame) {
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

        if (CHRONO_NOW - t0 > TO_PERIOD(samplingFrequency)) {
            if ((float) hbCnt / (float) roundTotal >= percentThreshHold) {
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

    /**
     * Construct a TDRD bodyframe command to be issued. Commands the current robot to naively approach the ball
     * without any obstacle avoidance enabled. The command will interpolate the movement of the ball so that
     * the robot "intercepts" the moving ball.
     * @param isHoldingBal == isEnabled && isCaptured
     * @param ballData
     * @param botData
     * @param interpolationRate The rate which we interpolate the movement of the ball. A rate of 1 means
     *                          that we go to where the ball should be after 1 sec assuming the velocity
     *                          stays constant.
     * @return A command to be issued
     */
    MotionCommand autoBallCaptureSolution(bool isHoldingBall, const BallData &ballData, const BotData &botData,
                                          double const interpolationRate, double angle) override {
//        MotionCommand command;
//        if (!isHoldingBall) {
//            double deltaX = ballData.pos(0) - botData.pos(0);
//            double deltaY = ballData.pos(1) - botData.pos(1);
//            double angle;
//            // might be simplified by using std::atan2, but i'm too lazy to refactor this since this method might be replaced by a new solution soon
//            if (deltaX < 0.0001 && deltaX > -0.0001) {
//                if (deltaY > 0) {
//                    angle = 0;
//                } else {
//                    angle = 179.9;
//                }
//            } else {
//                // first quadrant
//                if (deltaY >= 0 && deltaX >= 0) {
//                    angle = std::atan(deltaY / deltaX) * 180.0 / 3.1415926 - 90;
//                }
//                    // second quadrant
//                else if (deltaY >= 0 && deltaX < 0) {
//                    angle = 90 - std::atan(deltaY / -deltaX) * 180.0 / 3.1415926;
//                }
//                    // fourth quadrant
//                else if (deltaY < 0 && deltaX >= 0) {
//                    angle = std::atan(deltaY / deltaX) * 180.0 / 3.1415926 - 90;
//                }
//                    // third quadrant
//                else {
//                    angle = 90 - std::atan(deltaY / -deltaX) * 180.0 / 3.1415926;
//                }
//            }
//            command.mode = CtrlMode::TDRD;
//            command.frame = ReferenceFrame::BodyFrame;
//            command.setpoint3d = {ballData.pos(0), ballData.pos(1), angle + botData.ang};
//        } else {
//            command.mode = CtrlMode::TVRV;
//            command.frame = ReferenceFrame::BodyFrame;
//            command.setpoint3d = {0, 0, 0};
//        }
//        return command;

        MotionCommand command;
        if (!isHoldingBall) {
            double angleInRad = to_radian(angle);
            arma::vec2 botOrientation = {-std::sin(angleInRad), std::cos(angleInRad)};
            arma::vec2 ballvel = ballData.vel;
            double const botStopBeforeBallDistance = 150;
            if (arma::norm(ballvel) > 2000) {
                ballvel = 2000 * arma::normalise(ballvel);
            }

            arma::vec2 interpolatedPosition = ballData.pos + interpolationRate * ballvel -
                                              botStopBeforeBallDistance * botOrientation; // Calculate offset


            arma::vec2 botToInterPosDistance = interpolatedPosition - botData.pos;


            command.frame = ReferenceFrame::BodyFrame;

            if (arma::norm(botToInterPosDistance) > 100) {
                command.mode = CtrlMode::NSTDRD;
                command.setpoint3d = { interpolatedPosition(0), interpolatedPosition(1), angle };
            } else {
                command.mode = CtrlMode::TDRD;
                command.setpoint3d = { ballData.pos(0), ballData.pos(1), angle };
            }
        } else {
            command.mode = CtrlMode::TVRV;
            command.frame = ReferenceFrame::BodyFrame;
            command.setpoint3d = {0, 0, 0};
        }
        return command;
    }
};


class ErForceSimBotConfig : public VirtualBotConfig {
public:
    ErForceSimBotConfig() : VirtualBotConfig() { type = "ErForceSimBotConfig"; }

    bool isBallCloseEnoughToBot(const BallData &ballData, const BotData &botData) {
        // To-do
        return false;
    }

    MotionCommand
    autoBallCaptureSolution(bool isHoldingBall, const BallData &ballData, const BotData &botData,
                            double const interpolationRate, double angle) {
        // To-do
        return defaultMotionCommand();
    }
};


void processIni(std::string filepath, std::shared_ptr<BotConfig> config);