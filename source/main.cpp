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
#include "CoreModules/EKF-Module/MotionEkfModule.hpp"
#include "CoreModules/EKF-Module/BallEkfModule.hpp"
#include "CoreModules/ControlModule/ControlModule.hpp"
#include "CoreModules/MotionModule/MotionModule.hpp"
#include "CoreModules/BallCaptureModule/BallCaptureModule.hpp"
#include "PeriphModules/RemoteServers/TcpReceiveModule.hpp"
#include "PeriphModules/RemoteServers/UdpReceiveModule.hpp"
#include "PeriphModules/CameraClientModule/CameraClientModule.hpp"
//////////////////////////////////////////////////////////



std::ostream& operator<<(std::ostream& os, const arma::vec& v);

int main(int argc, char *argv[]) {
    // Logger Initialization
    BLogger::staticInit();
    BLogger::setToShorterFormat();
    BLogger::sink->set_filter(severity >= Info);
    
    // Process Json configurations


    // Process Comandline Arguments
    //bool isTestMode = false;
    //bool isVirtual = processArgs(argc, argv, isTestMode);
    CliConfig cliConfig = processArgs(argc, argv);
    std::shared_ptr<BotConfig> botConfig;
    if(cliConfig.isVirtual) {
        if(cliConfig.simulatorName == "grSim") {
            botConfig = std::unique_ptr<BotConfig>(new GrSimBotConfig());
        }
        if(cliConfig.simulatorName == "ErForce") {
            botConfig = std::unique_ptr<BotConfig>(new ErForceSimBotConfig());     
        }
    } else {
        // WIP
        return -1;
    }

    Config cfg(cliConfig, botConfig);


    // Preallocate Threads 
    ThreadPool threadPool(THREAD_POOL_SIZE); // pre-allocate # threads in a pool

    if(cliConfig.isTestMode) {
        TestRunner testRunner(cfg);
        testRunner.run(threadPool);
        threadPool.stopIosRun();
    } else {
        /*

        // Note: these smart pointer will be freed when exiting this else block (might cause seg fault if not dealt properly, java is so awesome)
        // Construct module instances
        std::unique_ptr<MotionEKF_Module> motionEkfModule(new VirtualMotionEKF());
        std::unique_ptr<BallEKF_Module> ballEkfModule(new VirtualBallEKF());
        std::unique_ptr<MotionModule> motionModule(new MotionModule());
        std::unique_ptr<ControlModule> controlModule(new PID_System());
        std::unique_ptr<UdpReceiveModule> udpReceiveModule(new UdpReceiveModule());
        std::unique_ptr<TcpReceiveModule> tcpReceiveModule(new TcpReceiveModule());
        std::unique_ptr<BallCaptureModule> ballCaptureModule(new BallCaptureModule());
        std::unique_ptr<CameraClientModule> cameraClientModule(new VirtualCameraClientModule());
        
        
        // Configs
        PID_System::PID_Constants pid_consts;
        pid_consts.RD_Kp = PID_RD_KP;   pid_consts.RD_Ki = PID_RD_KI;   pid_consts.RD_Kd = PID_RD_KD;
        pid_consts.TD_Kp = PID_TD_KP;   pid_consts.TD_Ki = PID_TD_KI;   pid_consts.TD_Kd = PID_TD_KD;
        ITPS::FieldPublisher<PID_System::PID_Constants> pidConstPub("PID", "Constants", pid_consts);

        // Run the servers
        //firmClientModule->run(threadPool);
        //motionEkfModule->run(threadPool);    
        ballEkfModule->run(threadPool);
        ///motionModule->run(threadPool);
        
        //controlModule->run(threadPool);
        //udpReceiveModule->run(threadPool);
        //tcpReceiveModule->run(threadPool);
        //ballCaptureModule->run(threadPool);


        threadPool.joinAll(); // must have it here, or will cause seg fault, think about the scope issue of the smart pointers
        */
    }

    threadPool.joinAll();
    return 0;
}

std::ostream& operator<<(std::ostream& os, const arma::vec& v)
{
    char fmt_str[30]; // not so important size, greater than printed str size is fine, use magic number here 
    int num_rows = arma::size(v).n_rows;
    os << "<";
    for(int i = 0; i < num_rows; i++) {
        sprintf(fmt_str, "%8.3lf", v(i));
        os << std::string(fmt_str);
        if(i != num_rows - 1) os << ", ";
    }
    os << ">";
    return os;
}
