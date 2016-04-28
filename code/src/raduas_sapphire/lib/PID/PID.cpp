/**
 * Created by Alex Bennett
 * http://alexben.net/
 *
 * Version: 0.1
 * File created: 04/18/2016
 * License: MIT
 */

#include "PID.h"

PID::PID(double *input_ptr, double *output_ptr, double set_point, double kp, double ki, double kd, int lower_limit, int upper_limit)
{
    this->_input_ptr = input_ptr;
}

void PID::update()
{
    // do nothing
}
