#pragma once
#include <armadillo>
#include "Misc/Utility/Common.hpp"
#include "CoreModules/DataCmdTypes.hpp"





arma::vec2 transformWorldToBodyFrame(arma::vec2 robotOriginInWorld, float robotAng, arma::vec2 pointToTransformInWorld);
arma::vec2 DEPRECATED_transformWorldToBodyFrame(arma::vec2 origin, float orien, arma::vec2 point2d);