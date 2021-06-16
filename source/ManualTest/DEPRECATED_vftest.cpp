#include "ManualTest/DEPRECATED_vftest.hpp"
#include "Misc/PubSubSystem/Module.hpp"
#include <iostream>
#include <string>
#include "Misc/Utility/ClockUtil.hpp"
#include "Config/ModuleFrequencies.hpp"
#include "PeriphModules/RemoteServers/TcpReceiveModule.hpp"
#include "ManualTest/TestRunner.hpp"
#include <cassert>

#include "PeriphModules/RemoteServers/TcpReceiveModule.hpp"
#include "PeriphModules/RemoteServers/UdpReceiveModule.hpp"
#include "PeriphModules/CameraClientModule/CameraClientModule.hpp"
#include "PeriphModules/McuClientModule/DEPRECATED_VFirmClient.hpp"
#include "CoreModules/CommandProcessorModule/CommandProcessorModule.hpp"
#include "CoreModules/DataProcessorModule/DataProcessorModule.hpp"
#include "CoreModules/MotionControllerModule/MotionControllerModule.hpp"
#include "CoreModules/BallCaptureModule/BallCaptureModule.hpp"


bool VFTest::test(ThreadPool& threadPool) {

    std::shared_ptr<BotDataFusion> botFilter;
    std::shared_ptr<BallDataFusion> ballFilter;
    botFilter = std::shared_ptr<VirtualBotDataFusion>(new VirtualBotDataFusion());
    ballFilter = std::shared_ptr<VirtualBallDataFusion>(new VirtualBallDataFusion());
// Note: these smart pointer will be freed when exiting this else block (might cause seg fault if not dealt properly, java is so awesome)
    // Construct module instances
    std::unique_ptr<UdpReceiveModule> udpReceiveModule(new UdpReceiveModule(config));
    std::unique_ptr<CameraClientModule> cameraClientModule(new VirtualCameraClientModule());
    std::unique_ptr<DataProcessorModule> dataProcessorModule(new DataProcessorModule(*botFilter, *ballFilter, config));
    std::unique_ptr<CommandProcessorModule> commandProcessorModule(new CommandProcessorModule(config));
    std::unique_ptr<MotionControllerModule> motionControllerModule(new MotionControllerModule(config));
    std::unique_ptr<BallCaptureModule> ballCaptureModule(new BallCaptureModule(config));

    std::unique_ptr<VFirmClientModule> vFirmClientModule(new VFirmClientModule());


    // Run the servers
    udpReceiveModule->run(threadPool);
    cameraClientModule->run(threadPool);
    dataProcessorModule->run(threadPool);
    commandProcessorModule->run(threadPool);
    motionControllerModule->run(threadPool);
    ballCaptureModule->run(threadPool);
    vFirmClientModule->run(threadPool);

    
    while(true) delay(10000);
    return true;
}
