#include "mbed.h"
#include "PID.hpp"

PID::PID();

float PID::pos_type_pid(){
    error = set_point - actual
    integral += error * dt
    deriv = error / dt
    output = kp * error + ki * integral - kd * deriv
    return output;
}