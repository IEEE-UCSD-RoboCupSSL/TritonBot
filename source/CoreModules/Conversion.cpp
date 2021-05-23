#include "CoreModules/Conversion.hpp"



arma::vec transformWorldToBodyFrame(arma::vec robotOriginInWorld, float robotAng, arma::vec pointToTransformInWorld) {

    auto v1 = pointToTransformInWorld;
    auto v2 = -robotOriginInWorld;
    auto vt = v1 + v2;
    return rotationMatrix2D(-robotAng) * vt;
}


// for explaination of the math, check motion_module.cpp
arma::vec DEPRECATED_transformWorldToBodyFrame(arma::vec origin, float orien, arma::vec point2d) {
    arma::mat A = WorldtoBodyHomoTransMat(origin, orien); // world to body homogeneous transformation
    arma::vec p_homo_w = {point2d(0), point2d(1), 1}; // homogeneous point end with a 1 (vector end with a 0)
    arma::vec p_homo_b = A * p_homo_w; // apply transformation to get the same point represented in the body frame
    // if division factor is approx. eq to zero
    if(std::fabs(p_homo_b(2)) < 0.000001) {
        p_homo_b(2) = 0.000001;
    }
    // update setpoint to the setpoint in robot's perspective (cartesean coordinate)
    arma::vec p_cart_b = {p_homo_b(0)/p_homo_b(2), p_homo_b(1)/p_homo_b(2)}; // the division is to divide the scaling factor, according to rules of homogeneous coord systems
    return p_cart_b;
}