#include "ManualTest/CoreModuleTests/DataProcessorModuleTest.hpp"
#include "CoreModules/DataCmdTypes.hpp"
#include "DataFusion/BallDataFusion.hpp"
#include "DataFusion/BotDataFusion.hpp"
#include "CoreModules/DataProcessorModule/DataProcessorModule.hpp"
#include "Misc/Utility/Common.hpp"
#include "Misc/Utility/ClockUtil.hpp"
#include "Misc/Utility/BoostLogger.hpp"
#include "Config/Config.hpp"

bool DataProcessorModuleTest::test(ThreadPool& threadPool) {
    VirtualBotDataFusion botFilter;
    VirtualBallDataFusion ballFilter;

    DataProcessorModule dataProcModule(botFilter, ballFilter);

    // Mock
    ITPS::FieldPublisher<SslVisionData> receivedSslVisionDataPub("From:UdpReceiveModule", "SslVision:BotData&BallData(WorldFrame)", defaultSslVisionData());
    ITPS::FieldPublisher<arma::vec> robotOriginInWorldPub("From:TcpReceiveModule", "RobotOrigin(WorldFrame)", zeroVec2d());
    ITPS::FieldPublisher<McuSensorData> mcuSensorDataPub("From:McuClientModule", "McuSensorData(BodyFrame)", defaultMcuSensorData());
    ITPS::FieldPublisher<CameraData> cameraDataPub("From:CameraClientModule", "CameraData(BodyFrame)", defaultCameraData());
    
    ITPS::FieldSubscriber<BotData> filteredBotDataSub("From:DataProcessorModule", "BotData(BodyFrame)");
    ITPS::FieldSubscriber<BallData> filteredBallDataSub("From:DataProcessorModule", "BallData(BodyFrame)");
    



    dataProcModule.run(threadPool);

    try {
        filteredBotDataSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        filteredBallDataSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
    }
    catch(std::exception& e) {
        BLogger logger;
        logger.addTag("[DataProcessorModuleTest.cpp]");
        logger.log(Error, std::string(e.what()));
        std::exit(0);
    }


    delay(500);

    std::cout << "test started" << std::endl;

    auto t0 = CHRONO_NOW;
    SslVisionData mockSslVisData = defaultSslVisionData();
    mockSslVisData.ballData.pos = {10, 10};
    mockSslVisData.botData.pos = {0, 50};
    mockSslVisData.botData.frame = ReferenceFrame::WorldFrame;

    float angles[8] = {0.0f, 45.0f, 90.0f, 135.0f, 180.0f, -135.0f, -90.0f, -45.0f};
    for(int i = 0; i < 8; i++) {
        mockSslVisData.botData.ang = angles[i];
        receivedSslVisionDataPub.publish(mockSslVisData);

        delay(10);

        std::cout << "processed-data: angle[" << filteredBotDataSub.latest_msg().ang << "] ";
        auto botPos = filteredBotDataSub.latest_msg().pos;
        auto ballPos = filteredBallDataSub.latest_msg().pos;
        std::cout << "botpos[" << botPos(0) << "," << botPos(1) << "] ";
        std::cout << "ballpos[" << ballPos(0) << "," << ballPos(1) << "]" <<std::endl;
    }

    threadPool.joinAll();

    

    return true;
}