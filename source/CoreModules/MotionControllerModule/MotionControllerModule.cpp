#include "CoreModules/MotionControllerModule/MotionControllerModule.hpp"
#include "Config/ModuleFrequencies.hpp"
#include "CoreModules/DataCmdTypes.hpp"
#include "Misc/Utility/BoostLogger.hpp"
#include "Misc/Utility/Common.hpp"
#include "Config/Config.hpp"
#include "CoreModules/Conversion.hpp"
#include "CoreModules/MotionControllerModule/PIDAlgorithm.hpp"

void MotionControllerModule::task(ThreadPool& threadPool) {
    ITPS::FieldSubscriber<ControlInput> controlInputSub("From:CommandProcessorModule", "MotionControlInput");
    ITPS::FieldSubscriber<bool> safetyEnableSub("From:TcpReceiveModule", "SafetyEnable"); 
    ITPS::FieldSubscriber<BotData> filteredBotDataSub("From:DataProcessorModule", "BotData(BodyFrame)");
    
    ITPS::FieldPublisher<ControlOutput> controlOutputPub("From:MotionControllerModule", "MotionControlOutput", defaultControlOutput());

    BLogger logger;
    logger.addTag("MotionControllerModule");
    logger(Info) << "\033[0;32m Thread Started \033[0m";

    try {
        controlInputSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        safetyEnableSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        filteredBotDataSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
    }
    catch(std::exception& e) {
        BLogger logger;
        logger.addTag("[MotionControllerModule.cpp]");
        logger.log(Error, std::string(e.what()));
        std::exit(0);
    }

    logger(Info) << "\033[0;32m Initialized \033[0m";


    // Construct PID controllers
   
    PIDController<float> anglePid(config.botConfig->anglePidConsts.toArmaVec3());
    PIDController<arma::vec2> posPid(config.botConfig->posPidConsts.toArmaVec3());

    while(true) {
        anglePid.init(config.botConfig->pidControlFrequency);
        posPid.init(config.botConfig->pidControlFrequency);
        while(safetyEnableSub.getMsg()) {
            periodic_session([&](){
                auto input = controlInputSub.getMsg();
                auto feedback = filteredBotDataSub.getMsg();
                ControlOutput output;
                float anglePidOutput, angVelOutput, corrAngle;
                arma::vec2 posPidOutput, velOutput;
                float pidAmplifier = 1.00;
                if(input.isNoSlowDownMode) {
                    pidAmplifier = config.botConfig->noSlowDownPidAmp;
                }
                anglePid.updatePidConsts(config.botConfig->anglePidConsts.toArmaVec3());
                posPid.updatePidConsts(pidAmplifier * config.botConfig->posPidConsts.Kp,  // only need to amplify Kp constant
                                                        config.botConfig->posPidConsts.Ki,
                                                        config.botConfig->posPidConsts.Kd);
                
                // PID calculations : Error = SetPoint - CurrPoint (i.e. ExpectedValue - ActualValue)
                // Rotational Movement Controller
                if(input.rotationalSetPoint.type == SetPointType::position) {
                    float angleErr = input.rotationalSetPoint.value - feedback.ang; // Expected Value - Actual Value
                    if(std::signbit(input.rotationalSetPoint.value) != std::signbit(feedback.ang)) {
                        // having opposite sign means one is in the 0 ~ 180 region and the other is in 0 ~ -180
                        float altError = angleErr > 0 ? (angleErr - 360) : (360 + angleErr);
                        // find the direction with the shortest angleErr value
                        if(std::fabs(angleErr) > std::fabs(altError)) {
                            angleErr = altError;
                        }
                    }
                    anglePidOutput = anglePid.calculate(angleErr);
                }
                else { // type == velocity
                    anglePid.init(config.botConfig->pidControlFrequency);
                    angVelOutput = input.rotationalSetPoint.value;
                }

                // Translational Movement Controller
                if(input.translationalSetPoint.type == SetPointType::position) {
                    // std::cout << "DEBUG: " << (input.translationalSetPoint.value - feedback.pos) << std::endl; 
                    posPidOutput = posPid.calculate(input.translationalSetPoint.value - feedback.pos );
                    // correct deviation due to rotation momentum
                    if(input.rotationalSetPoint.type == SetPointType::position) {
                        corrAngle = -anglePidOutput * config.botConfig->pidTdrdCorr;
                    }
                    else {
                        corrAngle = -angVelOutput * config.botConfig->pidTdrvCorr;
                    }
                    // posPidOutput = rotationMatrix2D(corrAngle) * posPidOutput; // correct direction by rotation matrix
                }
                else {
                    // type == velocity
                    posPid.init(config.botConfig->pidControlFrequency);
                    velOutput = input.translationalSetPoint.value;
                    // correct deviation due to rotation momentum
                    if(input.rotationalSetPoint.type == SetPointType::position) {
                        corrAngle = -anglePidOutput * config.botConfig->pidTvrdCorr;
                    }
                    else {
                        corrAngle = -angVelOutput * config.botConfig->pidTvrvCorr;
                    }
                    velOutput = rotationMatrix2D(corrAngle) * velOutput; // correct direction by rotation matrix
                }


                /* PID output selections
                 * we assume velocity and displacement are independent,
                 * so generally they should be mutually exclusive.
                 *
                 * translational and rotational variables are linear independent,
                 * so they can have different controllers running at the same time
                 */
                if(input.translationalSetPoint.type == SetPointType::position) {
                    //Note: pid controller takes in a pos and outputs a velocity robot needs to acheive in order to move to that position  
                    output.vx = posPidOutput(0); 
                    output.vy = posPidOutput(1);
                } else if(input.translationalSetPoint.type == SetPointType::velocity) {
                    output.vx = velOutput(0);
                    output.vy = velOutput(1);
                }
                if(input.rotationalSetPoint.type == SetPointType::position) {
                    output.omega = anglePidOutput;
                } else if(input.rotationalSetPoint.type == SetPointType::velocity) {
                    output.omega = angVelOutput;
                }


                /* Effect of Normalizing: more power spent on rotation results in less spent on translation, vice versa */
                /* Imaging the output to be a 3D vector: <x, y, z>, where z is omega, the angular speed 
                 * the constraint is a sphere in R3 with radius 100, corresponding to 100%.
                 * The robot should prioritize rotation so that it won't move crooked as often,
                 * hence, the constraint of <x, y> in R2 is pre-determined by the magnitude of the 
                 * rotational element 'z' (a.k.a w/omega), this constraint is the intersection between 
                 * the plaine z=outputArmaVec3(2) and the sphere.
                 * This percentage representation maps robot motion vector in a sphere whose projection on
                 * to the x-y plane is a circle, however the exact velocity constraint isn't a circle,
                 * but a 2D diamond determined by robot's max vertical and max horizontal speed, the mapping
                 * from the circle constraint to the diamond constraint will be taken care in the McuTop program
                 */
                auto outputArmaVec3 = output.toArmaVec3();
                float z = outputArmaVec3(2);
                if(z > 100.00f) z = 100.00f;
                if(z < -100.00f) z = -100.00f;
                //max magnitude for xy plain vector, which is the xy constraint: intersection between the sphere and the plaine z=outputArmaVec3(2)
                float maxMagForXy = std::sqrt(std::pow(100.00f, 2) - std::pow(z, 2));
                arma::vec2 xy = {outputArmaVec3(0), outputArmaVec3(1)};
                if(arma::norm(xy) > maxMagForXy) {
                    // Normalize the output vector to limit the maximum output vector norm to be maxMag
                    xy = arma::normalise(xy);
                    xy *= maxMagForXy;
                }
                output.vx = xy(0);
                output.vy = xy(1);
                output.omega = z;
                
                // publish output
                controlOutputPub.publish(output);

            }, TO_PERIOD(config.botConfig->pidControlFrequency));
        }
    }
}
