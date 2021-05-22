#pragma once
#include "CoreModules/DataCmdTypes.hpp"

class BallDataFusion {
public:
    virtual BallData calc(BallData sslVisionBallData, CameraData cameraDataa) = 0;
};


class VirtualBallDataFusion : BallDataFusion {
public:
    BallData calc(BallData sslVisionBallData, CameraData cameraData);
};