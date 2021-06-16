#pragma once

#include <armadillo>
#include <string>
#include <sstream>
#include "Misc/Utility/Common.hpp"

// Controller controls one of the following physics variables 
enum CtrlMode {TDRD = 0, //TDRD: translational displacement & rotational displacement (at the same time)
                TDRV = 1, //TDRV: translational displacement & rotational velocity (at the same time)
                TVRD = 2, //TVRD: translational velocity & rotational displacement (at the same time)
                TVRV = 3, //TVRV: translational velocity & rotational velocity (at the same time)  
                NSTDRD = 4, // No slowdown TDRD 
                NSTDRV = 5  // No slowdown TDRV
                };

/* IMPORTANT!!!
 * WorldFrame: global(player perspective) coordinate representation with the center of the field 
 *             being (0, 0), x axis on the horizontal width line of the field, 
 *             and y axis on the vertical length line of the field. The positive y axis points
 *             at the opponent goal.
 * 
 * BodyFrame: local coordinate representation with the INITIAL robot geometry center 
 *            being (0, 0), x axis on the robot's horizontal, y axis on the vertical.
 *            The positive y axis of BodyFrame points at the front of the robot at which
 *            the dribbler is. Note that point (0, 0) in BodyFrame refers to the static
 *            location of the robot when first initialized, which is DIFFERENT(!!!) than the 
 *            common notion of BodyFrame's (0, 0) which conventionally defined as the
 *            robot's center that moves as robot moves. The reason for choosing static
 *            origin is for avoiding the complications involved in having a moving coordinate
 *            system especially when dealing with a small displacement segment.   
 * 
 * Note: reference frame is only meaningful for vector representation of 
 *       a robot's position/velocity. The angle orientation of the robot
 *       has nothing to do with the frame of reference, the 0 degree orientation
 *       of the robot's font always points at the WorldFrame postive y axis direction.
 */
enum ReferenceFrame {WorldFrame = 0, BodyFrame = 1, NotDetermined = 2}; 


struct MotionCommand {
    arma::vec3 setpoint3d; /* <x, y, theta> where theta is the orientation angle, 
                              units: mm and degree (not radians!), degree range from (-180.0f ~ 180.0f degrees)
                              if the mode is of velocity control type, then the
                              unit for velocity <x, y> is the percentage of its max 
                              velocity on the corresponding axis, numerically : (-100.0f ~ 100.0f), where 100.0f represent 100% of max velocity at x or y
                               */
    CtrlMode mode;
    ReferenceFrame frame;
};

struct Command {
    MotionCommand motionCommand;
    bool enAutoCap;
    arma::vec2 kickerSetPoint; // unit: m/s
    std::string toString() {
        std::stringstream ss;
        ss << "\nCommand [\n" 
           << "\tMotionCommand [\n"
           << "\t\tSetPoint: " << motionCommand.setpoint3d << "\n"
           << "\t\tMode: " << motionCommand.mode << "\n"
           << "\t\tReferenceFrame: " << motionCommand.frame << "\n"
           << "\t]\n"
           << "\tKickerSetPoint: " << kickerSetPoint << "\n"
           << "\tEnAutoCap: " << (enAutoCap ? "True" : "False") << "\n"
           << "]" << std::endl; 
        return ss.str();
    }
};


struct BotData {  
    arma::vec2 pos;
    arma::vec2 vel;
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
    arma::vec2 pos; 
    arma::vec2 vel;  
    ReferenceFrame frame;
};


struct SslVisionData {
    BotData botData;
    BallData ballData;
    std::string toString() {
        std::stringstream ss;
        ss << "\nSSL Vision Data [\n"
           << "\tBotData [\n"
           << "\t\tPos: " << botData.pos << "\n"
           << "\t\tVel: " << botData.vel << "\n"
           << "\t\tAng: " << botData.ang << "\n"
           << "\t\tAngVel: " << botData.angVel << "\n"
           << "\t\tReferenceFrame: " << botData.frame << "\n"
           << "\t]\n"
           << "\tBallData [\n"
           << "\t\tPos: " << ballData.pos << "\n"
           << "\t\tVel: " << ballData.vel << "\n"
           << "\t\tReferenceFrame: " << ballData.frame << "\n"
           << "\t]\n"
           << "]" << std::endl; 
        return ss.str();
    }
};

struct McuSensorData {
    arma::vec2 encCnt;
    float imuTheta;
    float imuOmega;
    arma::vec2 imuAcc;
    bool isHoldingBall;
        std::string toString() {
        std::stringstream ss;
        ss << "\nMcuSensorData [\n"
           << "\tEncCount: " << encCnt << "\n"
           << "\tImuTheta: " << imuTheta << "\n"
           << "\tImuOmega: " << imuOmega << "\n"
           << "\tImuAcc: " << imuAcc << "\n"  
           << "\tIsHoldingBall: " << (isHoldingBall? "True" : "False") << "\n"   
           << "]" << std::endl; 
        return ss.str();
    }
    
};

struct CameraData {
    BallData ballData;
    bool isBallInFov;
    // ...
};


enum SetPointType {velocity, position};
template <typename ValueType>
struct SetPoint {
    ValueType value;
    SetPointType type;
};

struct ControlInput {
    SetPoint<arma::vec2> translationalSetPoint;
    SetPoint<float> rotationalSetPoint;
    bool isNoSlowDownMode;
    std::string toString() {
        std::stringstream ss;
        ss << "\nControlInput [\n"
           << "\tTranslationalSetPoint [\n"
           << "\t\tType: " << translationalSetPoint.type << "\n"
           << "\t\tValue: " << translationalSetPoint.value << "\n"
           << "\t]\n"
           << "\tRotationalSetPoint [\n"
           << "\t\tType: " << rotationalSetPoint.type << "\n"
           << "\t\tValue: " << rotationalSetPoint.value << "\n"
           << "\t]\n"
           << "]" << std::endl; 
        return ss.str();
    }
};

struct ControlOutput  {
    double vx, vy, omega;
    arma::vec3 toArmaVec3() {
        arma::vec3 v = {vx, vy, omega};
        return v;
    }
    std::string toString() {
        std::stringstream ss;
        ss << "ControlOutput [" << toArmaVec3() << "]" << std::endl; 
        return ss.str();
    }
};


struct PIDConstants {
    double Kp = 0, Ki = 0, Kd = 0;
    arma::vec3 toArmaVec3() {
        arma::vec3 v = {Kp, Ki, Kd};
        return v;
    }
    std::string toString() {
        std::stringstream ss;
        ss << "PIDConstants [" << toArmaVec3() << "]" << std::endl; 
        return ss.str();
    }
};



MotionCommand defaultMotionCommand();
Command defaultCommand();
BotData defaultBotData();
BallData defaultBallData();
SslVisionData defaultSslVisionData();
McuSensorData defaultMcuSensorData();
CameraData defaultCameraData();
ControlInput defaultControlInput();
PIDConstants defaultPIDConstants();
ControlOutput defaultControlOutput();