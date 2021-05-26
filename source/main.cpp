#include <iostream>
#include <armadillo>

#include "Misc/PubSubSystem/ThreadPool.hpp"
#include "Misc/Utility/BoostLogger.hpp"
#include "Misc/Utility/ClockUtil.hpp"
#include "Misc/Utility/Common.hpp"
#include "ProtoGenerated/vFirmware_API.pb.h"
#include "ManualTest/TestRunner.hpp"
#include "Config/Config.hpp"
#include "Config/CliConfig.hpp"
#include "Config/BotConfig.hpp"

////////////////////////MODULES///////////////////////////
#include "PeriphModules/RemoteServers/TcpReceiveModule.hpp"
#include "PeriphModules/RemoteServers/UdpReceiveModule.hpp"
#include "PeriphModules/CameraClientModule/CameraClientModule.hpp"
#include "PeriphModules/McuClientModule/DEPRECATED_VFirmClient.hpp"
#include "CoreModules/CommandProcessorModule/CommandProcessorModule.hpp"
#include "CoreModules/DataProcessorModule/DataProcessorModule.hpp"
#include "CoreModules/MotionControllerModule/MotionControllerModule.hpp"
#include "CoreModules/BallCaptureModule/BallCaptureModule.hpp"
//////////////////////////////////////////////////////////





int main(int argc, char *argv[]) {
    // Logger Initialization
    BLogger::staticInit();
    BLogger::setToShorterFormat();
    BLogger::sink->set_filter(severity >= Info);
    
    // Process Comandline Arguments
    CliConfig cliConfig = processArgs(argc, argv);
    std::shared_ptr<BotConfig> botConfig;

    // Process .ini config
    std::shared_ptr<BotDataFusion> botFilter;
    std::shared_ptr<BallDataFusion> ballFilter;
    if(cliConfig.isVirtual) {
        if(cliConfig.simulatorName == "grSim") {
            botConfig = std::shared_ptr<GrSimBotConfig>(new GrSimBotConfig());
        }
        if(cliConfig.simulatorName == "ErForce") {
            botConfig = std::shared_ptr<ErForceSimBotConfig>(new ErForceSimBotConfig());     
        }
        botFilter = std::shared_ptr<VirtualBotDataFusion>(new VirtualBotDataFusion());
        ballFilter = std::shared_ptr<VirtualBallDataFusion>(new VirtualBallDataFusion());
    } else {
        // WIP
        return -1;
    }
    if(cliConfig.botConfigFilePath != "") {
        processIni(cliConfig.botConfigFilePath, botConfig);
    }


    // Construct the all-in-one config object 
    Config cfg(cliConfig, botConfig);


    // Preallocate Threads 
    ThreadPool threadPool(THREAD_POOL_SIZE); // pre-allocate # threads in a pool

    


    if(cliConfig.isTestMode) {
        TestRunner testRunner(cfg);
        testRunner.run(threadPool);
        threadPool.stopIosRun();
    } else {
        if(cliConfig.isVirtual) {
            // Note: these smart pointer will be freed when exiting this else block (might cause seg fault if not dealt properly, java is so awesome)
            // Construct module instances
            std::unique_ptr<TcpReceiveModule> tcpReceiveModule(new TcpReceiveModule(cfg));
            std::unique_ptr<UdpReceiveModule> udpReceiveModule(new UdpReceiveModule(cfg));
            std::unique_ptr<CameraClientModule> cameraClientModule(new VirtualCameraClientModule());
            std::unique_ptr<DataProcessorModule> dataProcessorModule(new DataProcessorModule(*botFilter, *ballFilter, cfg));
            std::unique_ptr<CommandProcessorModule> commandProcessorModule(new CommandProcessorModule(cfg));
            std::unique_ptr<MotionControllerModule> motionControllerModule(new MotionControllerModule(cfg));
            std::unique_ptr<BallCaptureModule> ballCaptureModule(new BallCaptureModule(cfg));


        
            // Run the servers
            tcpReceiveModule->run(threadPool);
            udpReceiveModule->run(threadPool);
            cameraClientModule->run(threadPool);
            dataProcessorModule->run(threadPool);
            commandProcessorModule->run(threadPool);
            motionControllerModule->run(threadPool);
            ballCaptureModule->run(threadPool);
            


            threadPool.joinAll(); // must have it here, or will cause seg fault, think about the scope issue of the smart pointers
        } else {
            // To-do ...
            return -1;
        }
    }

    threadPool.joinAll();
    return 0;
}


