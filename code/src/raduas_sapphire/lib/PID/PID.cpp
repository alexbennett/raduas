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
    this->_output_ptr = output_ptr;
    this->_set_point = set_point;
    this->_kp = kp;
    this->_ki = ki;
    this->_kd = kd;
    this->_lower_limit = lower_limit;
    this->_upper_limit = upper_limit;
}

/**
 * Updates the PID output by running the full PID calculation.
 */
void PID::update()
{
    // First calculate current error
    double error = this->_set_point - *this->_input_ptr;

    // Update error over time (integration term)
    this->_integration_term += error;

    // Calculate P value
    double p_value = this->_kp * error;

    // Calculate I value
    double i_value = this->_ki * this->_integration_term;

    // Verify I value
    if(i_value > this->_upper_limit)
    {
        i_value = this->_upper_limit;
    }
    else if(i_value < this->_lower_limit)
    {
        i_value = this->_lower_limit;
    }

    // Calculate D value
    double d_value = this->_kd * (*this->_input_ptr - this->_previous_input);

    // Set the output
    *this->_output_ptr = p_value + i_value + d_value;

    // Update necessary value storage
    this->_previous_input = *this->_input_ptr;
}

/**
 * Sets the target setpoint of the PID.
 * @param set_point the setpoint to apply.
 */
void PID::setSetpoint(double set_point)
{
    this->_set_point = set_point;
}

/**
 * Sets the Kp multiplier of the PID.
 * @param kp the Kp to apply.
 */
void PID::setKp(double kp)
{
    this->_kp = kp;
}

/**
 * Sets the Ki multiplier of the PID.
 * @param ki the Ki to apply.
 */
void PID::setKi(double ki)
{
    this->_ki = ki;
}

/**
 * Sets the Kd multiplier of the PID.
 * @param kd the Kd to apply.
 */
void PID::setKd(double kd)
{
    this->_kd = kd;
}

/**
 * Sets lower and upper limits of the PID correction.
 * @param lower_limit the lower limit of the correction.
 * @param upper_limit the upper limit of the correction.
 */
void PID::setLimits(double lower_limit, double upper_limit)
{
    this->_lower_limit = lower_limit;
    this->_upper_limit = upper_limit;
}
