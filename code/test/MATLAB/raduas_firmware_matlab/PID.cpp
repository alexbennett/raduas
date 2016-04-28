#include "Arduino.h"
#include "PID.h"

PID::PID(float *input_ptr, float *output_ptr, float set_point, float Kp, float Ki, float Kd, float lower_limit, float upper_limit)
{
  // Set pointers to input and output
  _input_ptr = input_ptr;
  _output_ptr = output_ptr;

  // Set initial setpoint
  _set_point = set_point;

  // Set initial K parameters
  _Kp = Kp;
  _Ki = Ki;
  _Kd = Kd;

  // Set output limits
  _lower_limit = lower_limit;
  _upper_limit = upper_limit;

  // Set initial integration term
  _integration_term = 0;
}

void PID::calculate()
{
  // Grab input and calculate error
  float input = *_input_ptr;
  float error = _set_point - input;

  // Update integration term
  _integration_term += error;

  // Check that it is within bounds
  if(_integration_term > _upper_limit) _integration_term = _upper_limit;
  else if(_integration_term < _lower_limit) _integration_term = _lower_limit;

  // Calculate derivative
  float derivative = input - _previous_input;

  // Calculate PID output
  float output = (_Kp * error) + (_Ki * _integration_term) + (_Kd * derivative);

  // Check against max
  if(output > _upper_limit) output = _upper_limit;
  else if(output < _lower_limit) output = _lower_limit;
  
  // Update the output variable
  *_output_ptr = output;

  // Update miscellaneous value storage
  _previous_input = input;
}

void PID::setSetpoint(float set_point)
{
  _set_point = set_point;
}

void PID::setKp(float Kp)
{
  _Kp = Kp;
}

void PID::setKi(float Ki)
{
  _Ki = Ki;
}

void PID::setKd(float Kd)
{
  _Kd = Kd;
}

void PID::setLimits(float lower_limit, float upper_limit)
{
  _lower_limit = lower_limit;
  _upper_limit = upper_limit;
}

float PID::getKp()
{
  return _Kp;
}

float PID::getKi()
{
  return _Ki;
}

float PID::getKd()
{
  return _Kd;
}

