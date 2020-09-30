#pragma once

#include <armadillo>

class Field {
    public:
        Field() {} // To-Do

        static bool is_setpoint_out_of_play(arma::vec point) {
            return true; // To-Do
        }

        static arma::vec get_reflector_velocity(arma::vec curr_vel_setpoint) {
            return curr_vel_setpoint; // To-Do
        }
        
        static arma::vec get_amended_displacement(arma::vec curr_disp_setpoint) {
            return curr_disp_setpoint; // To-Do
        }

};

