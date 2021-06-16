#include "CoreModules/CommandProcessorModule/CommandProcessorModule.hpp"
#include "Config/ModuleFrequencies.hpp"
#include "CoreModules/DataCmdTypes.hpp"
#include "Misc/Utility/BoostLogger.hpp"
#include "Misc/Utility/Common.hpp"
#include "Config/Config.hpp"
#include "CoreModules/Conversion.hpp"


ControlInput processMotionCommand(const MotionCommand& receivedMCmd, arma::vec2 robotOriginInWorld, float robotAng); 

void CommandProcessorModule::task(ThreadPool& threadPool) {
    /*** Publisher setup ***/
    ITPS::FieldPublisher<bool> dribblerCommandPub("From:CommandProcessorModule", "dribblerSwitch", false);
    ITPS::FieldPublisher<arma::vec2> kickerSetPointPub("From:CommandProcessorModule", "KickerSetPoint(On/Off)", zeroVec2d());
    ITPS::FieldPublisher<bool> enableAutoCapturePub("From:CommandProcessorModule", "EnableAutoCapture", false);
    ITPS::FieldPublisher<ControlInput> controlInputPub("From:CommandProcessorModule", "MotionControlInput", defaultControlInput());

    /*** Subscriber setup ***/
    ITPS::FieldSubscriber<Command> receivedCommandSub("From:UdpReceiveModule", "Command");
    ITPS::FieldSubscriber<arma::vec2> robotOriginInWorldSub("From:TcpReceiveModule", "RobotOrigin(WorldFrame)");
    ITPS::FieldSubscriber<BotData> filteredBotDataSub("From:DataProcessorModule", "BotData(BodyFrame)");
    ITPS::FieldSubscriber<BallData> filteredBallDataSub("From:DataProcessorModule", "BallData(BodyFrame)");
    ITPS::FieldSubscriber<MotionCommand> ballAutoCapMotionCommandSub("From:BallAutoCaptureModule", "MotionCommand(BodyFrame)");

    BLogger logger;
    logger.addTag("CommandProcessorModule");
    logger(Info) << "\033[0;32m Thread Started \033[0m";


    try {
        receivedCommandSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        robotOriginInWorldSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        filteredBallDataSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        filteredBotDataSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        ballAutoCapMotionCommandSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
    }
    catch(std::exception& e) {
        BLogger logger;
        logger.addTag("[CommandProcessorModule.cpp]");
        logger.log(Error, std::string(e.what()));
        std::exit(0);
    }


    logger(Info) << "\033[0;32m Initialized \033[0m";

    MotionCommand mCmd;

    while(true) {
        periodic_session([&](){
            auto ballData = filteredBallDataSub.getMsg();
            auto botData = filteredBotDataSub.getMsg();
            auto receivedCmd = receivedCommandSub.getMsg();

            /** Automatically turning dribbler on when ball is close enough **/
            if(config.cliConfig.isVirtual) {
                auto bcfg = std::static_pointer_cast<VirtualBotConfig>(config.botConfig);
                dribblerCommandPub.publish(bcfg->isBallCloseEnoughToBot(ballData, botData));
            } else {
                // To-do: for real robots
            }

            /** send kicker setpoint directly **/
            kickerSetPointPub.publish(receivedCmd.kickerSetPoint);


            /** Process Ball AutoCapture or Bot RegularMotion **/
            if(receivedCmd.enAutoCap) {
                // AutoCaptureMode
                enableAutoCapturePub.publish(true);
                mCmd = ballAutoCapMotionCommandSub.getMsg();
            } else {
                // RegularMotionMode
                enableAutoCapturePub.publish(false);
                mCmd = receivedCmd.motionCommand;
            }

            /** publish calculated control input to the MotionController module **/
            auto ctrlIn = processMotionCommand(mCmd, robotOriginInWorldSub.getMsg(), botData.ang);
            controlInputPub.publish(ctrlIn);

        }, TO_PERIOD(COMMAND_PROCESSOR_FREQUENCY));
    }
}




ControlInput processMotionCommand(const MotionCommand& receivedMCmd, arma::vec2 robotOriginInWorld, float robotAng) { 
    ControlInput ci;
    arma::vec3 setpoint3d = receivedMCmd.setpoint3d;
    CtrlMode mode = receivedMCmd.mode;
    ReferenceFrame frame = receivedMCmd.frame;
    switch(mode) {
        case TDRD: ci.translationalSetPoint.type = SetPointType::position;
                   ci.rotationalSetPoint.type = SetPointType::position;
                   ci.isNoSlowDownMode = false;
                   break;
        case TDRV: ci.translationalSetPoint.type = SetPointType::position;
                   ci.rotationalSetPoint.type = SetPointType::velocity;
                   ci.isNoSlowDownMode = false;
                   break;
        case TVRD: ci.translationalSetPoint.type = SetPointType::velocity;
                   ci.rotationalSetPoint.type = SetPointType::position;
                   ci.isNoSlowDownMode = false;
                   break;
        case TVRV: ci.translationalSetPoint.type = SetPointType::velocity;
                   ci.rotationalSetPoint.type = SetPointType::velocity;
                   ci.isNoSlowDownMode = false;
                   break;
        case NSTDRD: ci.translationalSetPoint.type = SetPointType::position;
                     ci.rotationalSetPoint.type = SetPointType::position;
                     ci.isNoSlowDownMode = true;
                     break;
        case NSTDRV: ci.translationalSetPoint.type = SetPointType::position;
                     ci.rotationalSetPoint.type = SetPointType::velocity;
                     ci.isNoSlowDownMode = true;
                     break;
    }

    ci.translationalSetPoint.value = {setpoint3d(0), setpoint3d(1)};
    ci.rotationalSetPoint.value = setpoint3d(2);


    // If CMD setpoint is described in World Reference Frame, we do a World to Body Transformation
//    if(frame == WorldFrame) {
        if(ci.translationalSetPoint.type == SetPointType::position) {
            ci.translationalSetPoint.value = transformWorldToBodyFrame(robotOriginInWorld, 
                                robotAng, ci.translationalSetPoint.value);
        } else {
            ci.translationalSetPoint.value = transformWorldToBodyFrame(zeroVec2d(), 
                                robotAng, ci.translationalSetPoint.value);
        }
//    }
    return ci;
}
