#pragma once
#include <armadillo>
#include "Misc/Utility/Common.hpp"
#include "CoreModules/DataCmdTypes.hpp"





arma::vec transformWorldToBodyFrame(arma::vec robotOriginInWorld, float robotAng, arma::vec pointToTransformInWorld);
arma::vec DEPRECATED_transformWorldToBodyFrame(arma::vec origin, float orien, arma::vec point2d);