#include "Misc/Utility/Common.hpp"

double map(double value, range_t from, range_t to) {
    if(value < from.first) return to.first;
    if(value > from.second) return to.second;
    double percentage = (value - from.first) / (from.second - from.first); 
    return percentage * (to.second - to.first) + to.first;
}

// element-wise mapping
arma::vec map(arma::vec value, range_t from, range_t to) {
    for(int i = 0; i < size(value).n_rows; i++) {
        value(i) = map(value(i), from, to);
    }
    return value;
}

// pass by reference element-wise mapping, suit for large size vectors
void map2(arma::vec& value, range_t from, range_t to) {
    for(int i = 0; i < size(value).n_rows; i++) {
        value(i) = map(value(i), from, to);
    }
}

arma::mat rotation_matrix_2D(double angle_degree) {
    double x = to_radian(angle_degree);
    arma::mat rot = {{cos(x), -sin(x)},
               {sin(x),  cos(x)}};
    return rot;
}

// return mat that transforms from: 
//     standard basis [(1,0), (0,1)] ===> basis [\vec{vx}, \vec{vy}]relative to std basis
arma::mat change_basis_matrix_2D(arma::vec vx, arma::vec vy) {
    arma::mat P_inv = {{vx(0), vy(0)},
                 {vx(1), vy(1)}};
    return inv(P_inv);
}



/* Checkout this Lecture Slides http://ivl.calit2.net/wiki/images/4/4a/04_CoordinateSystemsF19.pdf 
 * world to body homogeneous transformation matrix : world <x, y> ==> body <u, v>
 *      (Homogeneous Transformation offers a nice way of transform both vector translation and rotation into one single matrix) 
 * robot_position_w : robot's 2d location on the field with respect to the world reference frame (or a.k.a Camera Frame) 
 * robot_orien_w    : robot's orientation with respect to the world reference frame
 * */
arma::mat wtb_homo_transform(arma::vec robot_position_w, double robot_orien_w) { 
    /* World frame to body frame transformation 3x3 matrix A for 2D vector, one extra dimension is demanded by homogenous transform
     * 
     * In homogenouse coordinates: a 2d position is written as <x, y, 1>^T, and a2d vector is written as <x, y, 0>^T, written in colunme vectors
     * 
     * point p = <p_x, p_y, 1>^T represented in world frame (basis x,y)
     * transformation A*p = p', where p' = <p'_u, p'_v, 1>^T describes the same point now using the body ref frame (basis u,v)
     * 
     * Homogeneous Transformation matrix
     * A = [Xu, Yu, Ou
     *      Xv, Yv, Ov   == [Xuv, Yuv, Ouv]  
     *       0,  0,  1]
     * where Ouv : <Ou, Ov, 1>^T is the origin point of the worldframe represented in the body frame coordinates
     *       Xuv : <Xu, Xv, 0>^T is the x unit vector of worldframe represented in body coord
     *       Yuv : <Yu, Yv, 0>^T is the y unit vector of worldframe represented in body coord  
     *  (hint: look up difference of vectors vs points in homogenouse coordinates)
     * 
     * 
     * In this case, we don't know Ouv, the origin point of worldframe in bodyframe coordinates,
     *             but we do know O'xy, the origin point of bodyframe in world coordinates, which is the robot position
     * similarly we can also calculate Uxy, Vxy:
     *      Uxy = rotation_matrix(theta) * <1, 0>^T then convert to homogeneous vector
     *      Vxy = rotation_matrix(theta) * <0, 1>^T then .....              (why standard basis vector? think about diff between vector vs point)
     *      where theta = bodyframe orientation - worldframe orientation 
     * Then we can construct the inverse of A in order to get A
     *  A_inv = [Uxy, Vxy, O'xy]
     *  A = inv(A_inv)
     * 
     * Note: Uxy, Vxy are vectors, O'xy is a point
     * 
     * detailed reference: http://ivl.calit2.net/wiki/images/4/4a/04_CoordinateSystemsF19.pdf
     */

    // get rotation matrix from robot's orientation
    arma::mat rot = rotation_matrix_2D(robot_orien_w);
    
    // these are regular vector, not homo vec, will convert them mannually in matrix construction
    arma::vec unit_vec_x = {1, 0};
    arma::vec unit_vec_y = {0, 1};
    arma::vec Oxy = robot_position_w; 
    arma::vec Uxy = rot * unit_vec_x;
    arma::vec Vxy = rot * unit_vec_y;

    arma::mat A_inv =  {{Uxy(0), Vxy(0), Oxy(0)},
                        {Uxy(1), Vxy(1), Oxy(1)},
                        {    0 ,     0 ,     1 }};

    return inv(A_inv);
}


arma::vec zeroVec2d(void) {
    arma::vec zero_vec = {0, 0};
    return zero_vec;
}
