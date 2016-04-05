#include "Arduino.h"
#include "PID.h"

PID::PID(float *input_ptr, float *output_ptr, float set_point, float Kp, float Ki, float Kd, float lower_limit, float upper_limit, float cycle_time)
{
  // Set pointers to input and output
  _input_ptr = input_ptr;
  _output_ptr = output_ptr;

  // Set initial setpoint
  setSetpoint(set_point);
  
  // Set cycle time
  setCycleTime(cycle_time);

  // Set initial K parameters
  setKp(Kp);
  setKi(Ki);
  setKd(Kd);

  // Set output limits
  _lower_limit = lower_limit;
  _upper_limit = upper_limit;
  
  // Set initial integration term
  _integration_term = 0;
}

void PID::calculate()
{
  unsigned long current_time = millis();

  // Only calculate if it's time for the next cycle  
  if((current_time - _previous_cycle_time) >= _cycle_time)
  {
    // Grab input and calculate error
    float input = *_input_ptr;
    float error = _set_point - input;
  
    // Update error sum
    _integration_term += (_Ki * error);
  
    // Check that it is within bounds
    if(_integration_term > _upper_limit) 
    {
      _integration_term = _upper_limit;
    }
    else if(_integration_term < _lower_limit) 
    {
      _integration_term = _lower_limit;
    }
  
    // Calculate derivative
    float derivative = input - _previous_input;
  
    // Calculate PID output
    float output = (_Kp * error) + (_integration_term) - (_Kd * derivative);
  
    // Check against max
    if(output > _upper_limit) output = _upper_limit;
    else if(output < _lower_limit) output = _lower_limit;
    
    // Update the output variable
    *_output_ptr = output;
  
    // Update miscellaneous value storage
    _previous_cycle_time = current_time;
    _previous_input = input;
  }
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
  if(lower_limit > upper_limit) return;
  
  _lower_limit = lower_limit;
  _upper_limit = upper_limit;
}

void PID::setCycleTime(float cycle_time)
{
  _cycle_time = cycle_time * 1000;
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

