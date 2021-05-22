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

/* WorldFrame: global(player perspective) coordinate representation with the center of the field 
 *             being (0, 0), x axis on the horizontal width line of the field, 
 *             and y axis on the vertical length line of the field. The positive y axis points
 *             at the opponent goal.
 * 
 * BodyFrame: local coordinate representation with the robot geometry center 
 *            being (0, 0), x axis on the robot's horizontal, y axis on the vertical.
 *            The positive y axis of BodyFrame points at the front of the robot at which
 *            the dribbler is.    
 * 
 * Note: reference frame is only meaningful for vector representation of 
 *       a robot's position/velocity. The angle orientation of the robot
 *       has nothing to do with the frame of reference, the 0 degree orientation
 *       of the robot's font always points at the WorldFrame postive y axis direction.
 */
enum ReferenceFrame {WorldFrame = 0, BodyFrame = 1, NotDetermined = 2}; 


struct MotionCommand {
    arma::vec setpoint3d; // <x, y, theta> where theta is the orientation angle 
    CtrlMode mode;
    ReferenceFrame frame;
};


struct BotData {  
    arma::vec pos;
    arma::vec vel;
    ReferenceFrame frame;

    /* Reference frame does not apply
     * to angular quantities.
     */
    float ang;   /* the orientation angle of the robot, 
                  * 0 degree aims at the postive WorldFrame y direction
                  * with counter-clock wise half being postive angle.
                  * i.e. (0 ~  180) being the left half circle, 
                  *      (0 ~ -180) being the right half circle
                  */
    float angVel; 
};

struct BallData { 
    arma::vec pos; 
    arma::vec vel;  
    ReferenceFrame frame;
};

MotionCommand defaultCmd();
BotData defaultBotData();
BallData defaultBallData();


