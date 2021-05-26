#pragma once
#include "CoreModules/DataCmdTypes.hpp"

class BotDataFusion {
public:
    virtual BotData calc(const BotData& sslVisionBotData, const McuSensorData& mcuSensorData) = 0;
};


class VirtualBotDataFusion : public BotDataFusion {
public:
    BotData calc(const BotData& sslVisionBotData, const McuSensorData& mcuSensorData);
};