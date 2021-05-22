#pragma once
#include <armadillo>
#include "Misc/Utility/Common.hpp"
#include "CoreModules/DataCmdTypes.hpp"





arma::vec transformWorldToBodyFrame(BotData botDataInWorldFrame, arma::vec pointToTransform);
arma::vec DEPRECATED_transformWorldToBodyFrame(arma::vec origin, float orien, arma::vec point2d);