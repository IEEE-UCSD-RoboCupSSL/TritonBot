#include <iostream>
#include <armadillo>

#include "Misc/PubSubSystem/ThreadPool.hpp"
#include "Misc/Utility/BoostLogger.hpp"
#include "Misc/Utility/Systime.hpp"
#include "Misc/Utility/Common.hpp"
#include "Config/Config.hpp"
#include "ProtoGenerated/vFirmware_API.pb.h"

////////////////////////MODULES///////////////////////////
#include "CoreModules/EKF-Module/MotionEkfModule.hpp"
#include "CoreModules/EKF-Module/BallEkfModule.hpp"
#include "CoreModules/ControlModule/ControlModule.hpp"
#include "CoreModules/MotionModule/MotionModule.hpp"
#include "CoreModules/BallCaptureModule/BallCaptureModule.hpp"
#include "PeriphModules/RemoteServers/TcpReceiveModule.hpp"
#include "PeriphModules/RemoteServers/UdpReceiveModule.hpp"
#include "PeriphModules/FirmClientModule/FirmClientModule.hpp"
//////////////////////////////////////////////////////////
#include "ManualTest/TestRunner.hpp"


std::ostream& operator<<(std::ostream& os, const arma::vec& v);

int main(int argc, char *argv[]) {
    // Logger Initialization
    BLogger::static_init();
    BLogger::set_shorter_format();
    BLogger::sink->set_filter(severity >= Info);
    BLogger logger;
    
    // Process Json configurations


    // Process Comandline Arguments
    bool isTestMode = false;
    bool isVirtual = processArgs(argc, argv, isTestMode);


    // Preallocate Threads 
    ThreadPool threadPool(THREAD_POOL_SIZE); // pre-allocate # threads in a pool

    if(isTestMode) {
        TestRunner testRunner;
        testRunner.run(threadPool);
    } else {
        // Construct module instances
        boost::shared_ptr<FirmClientModule> firmClientModule(new VFirmClient());
        boost::shared_ptr<MotionEKF_Module> motionEkfModule(new VirtualMotionEKF());
        boost::shared_ptr<BallEKF_Module> ballEkfModule(new VirtualBallEKF());
        boost::shared_ptr<MotionModule> motionModule(new MotionModule());
        boost::shared_ptr<ControlModule> controlModule(new PID_System());
        boost::shared_ptr<UdpReceiveModule> udpReceiveModule(new CMDServer());
        boost::shared_ptr<TcpReceiveModule> tcpReceiveModule(new ConnectionServer());
        boost::shared_ptr<BallCaptureModule> ballCaptureModule(new BallCaptureModule());
        
        // Configs
        PID_System::PID_Constants pid_consts;
        pid_consts.RD_Kp = PID_RD_KP;   pid_consts.RD_Ki = PID_RD_KI;   pid_consts.RD_Kd = PID_RD_KD;
        pid_consts.TD_Kp = PID_TD_KP;   pid_consts.TD_Ki = PID_TD_KI;   pid_consts.TD_Kd = PID_TD_KD;
        ITPS::NonBlockingPublisher<PID_System::PID_Constants> pidConstPub("PID", "Constants", pid_consts);

        // Run the servers
        firmClientModule->run(threadPool);
        motionEkfModule->run(threadPool);    
        ballEkfModule->run(threadPool);
        motionModule->run(threadPool);
        controlModule->run(threadPool);
        udpReceiveModule->run(threadPool);
        tcpReceiveModule->run(threadPool);
        // intern_ekf_server_module->run(threadPool);
        ballCaptureModule->run(threadPool);
    }

    while(1) { // has delay (good for reducing high CPU usage)
        // this program should run forever 
        delay(10000);
    }

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
