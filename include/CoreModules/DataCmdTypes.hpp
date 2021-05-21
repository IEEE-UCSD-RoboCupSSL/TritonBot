#pragma once

#include <armadillo>

// Controller controls one of the following physics variables 
enum CtrlMode {TDRD = 0, //TDRD: translational displacement & rotational displacement (at the same time)
                TDRV = 1, //TDRV: translational displacement & rotational velocity (at the same time)
                TVRD = 2, //TVRD: translational velocity & rotational displacement (at the same time)
                TVRV = 3, //TVRV: translational velocity & rotational velocity (at the same time)  
                NSTDRD = 4, // No slowdown TDRD 
                NSTDRV = 5  // No slowdown TDRV
                };
enum ReferenceFrame {WorldFrame = 0, BodyFrame = 1};

struct MotionCMD {
    arma::vec setpoint3d; // <x, y, \theta> where \theta is the orientation angle 
    CtrlMode mode;
    ReferenceFrame refFrame;
};


/* Motion frame: when facing yellow gate from blue gate, it is the positive x direction. The positive y direction
    * is the positive x direction rotated counter-clockwise by 90 degree */
struct BotData {
    arma::vec pos;
    arma::vec vel;
    float ang;   
    float angVel; 
};

struct BallData {
    arma::vec pos; // Location of ball relative to center of the field in ball frame.
    arma::vec vel;  // Velocity of ball in ball frame
};

MotionCMD defaultCmd();


