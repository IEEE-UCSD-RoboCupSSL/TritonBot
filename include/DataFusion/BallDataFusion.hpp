#pragma once
#include "CoreModules/DataCmdTypes.hpp"

class BallDataFusion {
public:
    virtual BallData calc(const BallData& sslVisionBallData, const CameraData& cameraDataa) = 0;
};


class VirtualBallDataFusion : public BallDataFusion {
public:
    BallData calc(const BallData& sslVisionBallData, const CameraData& cameraData);
};