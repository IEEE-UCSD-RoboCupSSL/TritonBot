#include "CoreModules/DataProcessorModule/DataProcessorModule.hpp"
#include "Config/ModuleFrequencies.hpp"
#include "CoreModules/DataCmdTypes.hpp"
#include "Misc/Utility/BoostLogger.hpp"
#include "Misc/Utility/Common.hpp"
#include "Config/Config.hpp"
#include "CoreModules/Conversion.hpp"



DataProcessorModule::DataProcessorModule(BotDataFusion& botdf, BallDataFusion& balldf) 
    : botFusion(&botdf), ballFusion(&balldf) {

}



BotData convertToBodyFrame(BotData botDataWorldFrame, arma::vec botOrigin) {
    BotData dataBodyFrame;
    dataBodyFrame.ang = botDataWorldFrame.ang;
    dataBodyFrame.angVel = botDataWorldFrame.angVel;
    dataBodyFrame.frame = ReferenceFrame::BodyFrame;
    dataBodyFrame.pos = transformWorldToBodyFrame(botOrigin, botDataWorldFrame.ang, botDataWorldFrame.pos);
    dataBodyFrame.vel = transformWorldToBodyFrame(botOrigin, botDataWorldFrame.ang, botDataWorldFrame.vel);
    return dataBodyFrame;
}

BallData convertToBodyFrame(BallData ballDataWorldFrame, arma::vec botOrigin, float botAng) {
    BallData dataBodyFrame;
    dataBodyFrame.frame = ReferenceFrame::BodyFrame;
    dataBodyFrame.pos = transformWorldToBodyFrame(botOrigin, botAng, ballDataWorldFrame.pos);
    dataBodyFrame.vel = transformWorldToBodyFrame(botOrigin, botAng, ballDataWorldFrame.vel);
    return dataBodyFrame;
}


void DataProcessorModule::task(ThreadPool& threadPool) {
    /*** Subscriber setup ***/
    ITPS::FieldSubscriber<Command> receivedCommandSub("From:UdpReceiveModule", "Command");
    ITPS::FieldSubscriber<SslVisionData> receivedSslVisionDataSub("From:UdpReceiveModule", "SslVision:BotData&BallData(WorldFrame)");
    ITPS::FieldSubscriber<arma::vec> robotOriginInWorldSub("From:TcpReceiveModule", "RobotOrigin(WorldFrame)");
    ITPS::FieldSubscriber<McuSensorData> mcuSensorDataSub("From:McuClientModule", "McuSensorData(BodyFrame)");
    ITPS::FieldSubscriber<CameraData> cameraDataSub("From:CameraModule", "CameraData(BodyFrame)");

    /*** Publisher setup ***/
    ITPS::FieldPublisher<BotData> filteredBotDataPub("From:DataProcessorModule", "BotData(BodyFrame)", defaultBotData());
    ITPS::FieldPublisher<BallData> filteredBallDataPub("From:DataProcessorModule", "BallData(BodyFrame)", defaultBallData());
    

    try {
        receivedCommandSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        receivedSslVisionDataSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        mcuSensorDataSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
        cameraDataSub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
    }
    catch(std::exception& e) {
        BLogger logger;
        logger.addTag("[CommandProcessorModule.cpp]");
        logger.log(Error, std::string(e.what()));
        std::exit(0);
    }


    BotData botDataBodyFrame;
    BallData ballDataBodyFrame;

    auto botFilter = botFusion;
    auto ballFilter = ballFusion;

    BotData filteredBotData;
    BallData filteredBallData;

    while(true) {
        periodic_session([&](){

            botDataBodyFrame = convertToBodyFrame(receivedSslVisionDataSub.latest_msg().botData, 
                                                robotOriginInWorldSub.latest_msg());
            ballDataBodyFrame = convertToBodyFrame(receivedSslVisionDataSub.latest_msg().ballData,
                                    robotOriginInWorldSub.latest_msg(), botDataBodyFrame.ang); 

            filteredBotData = botFilter->calc(botDataBodyFrame, mcuSensorDataSub.latest_msg());
            filteredBallData = ballFilter->calc(ballDataBodyFrame, cameraDataSub.latest_msg());

            filteredBotDataPub.publish(filteredBotData);
            filteredBallDataPub.publish(filteredBallData);


        }, TO_PERIOD(COMMAND_PROCESSOR_FREQUENCY));
    }



}

