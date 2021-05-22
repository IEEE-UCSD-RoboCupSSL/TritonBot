#include "PeriphModules/CameraClientModule/CameraClientModule.hpp"
#include <string>
#include <thread>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include "Misc/PubSubSystem/ThreadPool.hpp"
#include "Misc/Utility/ClockUtil.hpp"
#include "Misc/Utility/BoostLogger.hpp"
#include "Misc/Utility/Common.hpp"
#include "Config/Config.hpp"
#include "Config/ModuleFrequencies.hpp"


void VirtualCameraClientModule::task(ThreadPool& threadPool) {
    ITPS::FieldPublisher<CameraData> cameraDataPub("From:CameraClientModule", "CameraData(BodyFrame)", defaultCameraData());
    threadPool.joinAll();
}
