#ifndef PID_h
#define PID_h

#include "Arduino.h"

/**
 * Created by Alex Bennett
 * http://alexben.net/
 * 
 * Version: 0.1
 * File created: 12/02/2015
 * License: MIT
 */

class PID
{
  public:
    PID(float*, float*, float, float, float, float, float, float, float);

    void calculate();

    void setCycleTime(float);
    void setSetpoint(float);
    void setKp(float);
    void setKi(float);
    void setKd(float);
    void setLimits(float, float);

    float getKp();
    float getKi();
    float getKd();

  private:
    float _Kp, _Ki, _Kd;

    float _previous_input;
    float _integration_term;
    
    float *_input_ptr;
    float *_output_ptr;
    float _set_point;

    unsigned long _previous_cycle_time;
    float _cycle_time;

    float _lower_limit, _upper_limit;
};

#endif
