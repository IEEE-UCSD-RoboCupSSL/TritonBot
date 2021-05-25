#pragma once

#include <iostream>

#include <armadillo>
/*
 * Armadillo C++ library Citation:
 * 
 * Conrad Sanderson and Ryan Curtin.
 * Armadillo: a template-based C++ library for linear algebra.
 * Journal of Open Source Software, Vol. 1, pp. 26, 2016.
 *
 * Conrad Sanderson and Ryan Curtin.
 * A User-Friendly Hybrid Sparse Matrix Class in C++.
 * Lecture Notes in Computer Science (LNCS), Vol. 10931, pp. 422-430, 2018. 
 */



#include <math.h>
#include <unistd.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <boost/array.hpp>
#include <boost/signals2.hpp>
#include "BoostLogger.hpp"


/* Synchronization for Reader/Writer problems */
typedef boost::shared_mutex reader_writer_mutex; 
// get exclusive access
#define writer_lock(mutex) do { \
    boost::upgrade_lock<reader_writer_mutex> __writer_lock(mutex); \
    boost::upgrade_to_unique_lock<reader_writer_mutex> __unique_writer_lock( __writer_lock ); \
}while(0)
// get shared access
#define reader_lock(mutex) boost::shared_lock<reader_writer_mutex>  __reader_lock(mutex); 

#define repr std::to_string

#define UNUSED(expr) do { (void)(expr); } while (0)





using byte = unsigned char;
static constexpr const char* LOCAL_HOST = "127.0.0.1";

const float Pi = 3.1415926;

inline double to_radian(double deg) { return deg * Pi / 180.000;}
inline double to_degree(double rad) { return rad * (180.000 / Pi);}


using range_t = std::pair<double, double>; 

double map(double value, range_t from, range_t to);
arma::vec map(arma::vec value, range_t from, range_t to);
void map2(arma::vec& value, range_t from, range_t to);



arma::mat rotationMatrix2D(double angle_degree);
arma::mat changeBasisMatrix2D(arma::vec vx, arma::vec vy) ;
arma::mat WorldtoBodyHomoTransMat(arma::vec robot_position_w, double robot_orient_w);

arma::vec zeroVec2d(void);