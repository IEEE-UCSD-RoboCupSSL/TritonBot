#include "CoreModules/DataProcessorModule/DataProcessorModule.hpp"
#include "Config/ModuleFrequencies.hpp"
#include "CoreModules/DataCmdTypes.hpp"
#include "Misc/Utility/BoostLogger.hpp"
#include "Misc/Utility/Common.hpp"
#include "Config/Config.hpp"
#include "CoreModules/Conversion.hpp"

BotData convertToBodyFrame(const BotData& botDataWorldFrame, arma::vec2 botOrigin);
BallData convertToBodyFrame(const BallData& ballDataWorldFrame, arma::vec2 botOrigin, float botAng);


// config pass by copy on purpose to avoid multithreading synchronization concerns
DataProcessorModule::DataProcessorModule(BotDataFusion& botdf, BallDataFusion& balldf, Config cfg) 
    : botDataFilter(botdf), ballDataFilter(balldf), config(cfg) {

}

void DataProcessorModule::task(ThreadPool& threadPool) {
    /*** Subscriber setup ***/
    ITPS::FieldSubscriber<SslVisionData> receivedSslVisionDataSub("From:UdpReceiveModule", "SslVision:BotData&BallData(WorldFrame)");
    ITPS::FieldSubscriber<arma::vec2> robotOriginInWorldSub("From:TcpReceiveModule", "RobotOrigin(WorldFrame)");
    ITPS::FieldSubscriber<McuSensorData> mcuSensorDataSub("From:McuClientModule", "McuSensorData(BodyFrame)");
    ITPS::FieldSubscriber<CameraData> cameraDataSub("From:CameraClientModule", "CameraData(BodyFrame)");

    /*** Publisher setup ***/
    ITPS::FieldPublisher<BotData> filteredBotDataPub("From:DataProcessorModule", "BotData(BodyFrame)", defaultBotData());
    ITPS::FieldPublisher<BallData> filteredBallDataPub("From:DataProcessorModule", "BallData(BodyFrame)", defaultBallData());
    ITPS::FieldPublisher<bool> isHoldingBallPub("From:DataProcessorModule", "IsHoldingBall", false);


    BLogger logger;
    logger.addTag("DataProcessorModule");
    logger(Info) << "\033[0;32m Thread Started \033[0m";


    try {
        receivedSslVisionDataSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        mcuSensorDataSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        cameraDataSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        robotOriginInWorldSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
    }
    catch(std::exception& e) {
        BLogger logger;
        logger.addTag("[DataProcessorModule.cpp]");
        logger.log(Error, std::string(e.what()));
        std::exit(0);
    }


    logger(Info) << "\033[0;32m Initialized \033[0m";


    BotData botDataBodyFrame;
    BallData ballDataBodyFrame;


    BotData filteredBotData;
    BallData filteredBallData;

    while(true) {
        periodic_session([&](){
            auto sslVisionData = receivedSslVisionDataSub.getMsg();
            auto origin = robotOriginInWorldSub.getMsg();

            botDataBodyFrame = convertToBodyFrame(sslVisionData.botData, 
                                                robotOriginInWorldSub.getMsg());
            ballDataBodyFrame = convertToBodyFrame(sslVisionData.ballData,
                                    origin, botDataBodyFrame.ang); 

            filteredBotData = botDataFilter.calc(botDataBodyFrame, mcuSensorDataSub.getMsg());
            filteredBallData = ballDataFilter.calc(ballDataBodyFrame, cameraDataSub.getMsg());

            filteredBotDataPub.publish(filteredBotData);
            filteredBallDataPub.publish(filteredBallData);

            if(config.cliConfig.isVirtual) {
                if(config.cliConfig.simulatorName == "grSim") {
                    auto bcfg = std::static_pointer_cast<GrSimBotConfig>(config.botConfig);
                    isHoldingBallPub.publish(
                        bcfg->isHoldingBall(filteredBallData, filteredBotData)
                    );
                }
                // ...
            } else {
                isHoldingBallPub.publish(mcuSensorDataSub.getMsg().isHoldingBall);
            }
        }, TO_PERIOD(DATA_PROCESSOR_FREQUENCY));
    }
}

BotData convertToBodyFrame(const BotData& botDataWorldFrame, arma::vec2 botOrigin) {
    BotData dataBodyFrame;
    dataBodyFrame.ang = botDataWorldFrame.ang;
    dataBodyFrame.angVel = botDataWorldFrame.angVel;
    dataBodyFrame.frame = ReferenceFrame::BodyFrame;
    dataBodyFrame.pos = transformWorldToBodyFrame(botOrigin, botDataWorldFrame.ang, botDataWorldFrame.pos);
    dataBodyFrame.vel = transformWorldToBodyFrame(zeroVec2d(), botDataWorldFrame.ang, botDataWorldFrame.vel);
    return dataBodyFrame;
}

BallData convertToBodyFrame(const BallData& ballDataWorldFrame, arma::vec2 botOrigin, float botAng) {
    BallData dataBodyFrame;
    dataBodyFrame.frame = ReferenceFrame::BodyFrame;
    dataBodyFrame.pos = transformWorldToBodyFrame(botOrigin, botAng, ballDataWorldFrame.pos);
    dataBodyFrame.vel = transformWorldToBodyFrame(zeroVec2d(), botAng, ballDataWorldFrame.vel);
    return dataBodyFrame;
}



