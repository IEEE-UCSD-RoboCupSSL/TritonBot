#pragma once
#include "CoreModules/DataCmdTypes.hpp"

class BotDataFusion {
public:
    virtual BotData calc(BotData& sslVisionBotData, McuSensorData& mcuSensorData) = 0;
};


class VirtualBotDataFusion : public BotDataFusion {
public:
    BotData calc(BotData& sslVisionBotData, McuSensorData& mcuSensorData);
};