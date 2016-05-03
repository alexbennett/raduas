/**
 * Created by Alex Bennett
 * http://alexben.net/
 *
 * Version: 0.1
 * File created: 04/18/2016
 * License: MIT
 */

#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID
{
public:
    PID(double*, double*, double, double, double, double, int, int);

    void update();

    void setSetpoint(double set_point);
    void setKp(double kp);
    void setKi(double ki);
    void setKd(double kd);
    void setLimits(double lower_limit, double upper_limit);

    void getSetpoint();
    void getKp();
    void getKi();
    void getKd();
    void getLowerLimit();
    void getUpperLimit();

private:
    double _set_point;

    double *_input_ptr;
    double *_output_ptr;

    double _kp, _ki, _kd;
    int _lower_limit, _upper_limit;

    double _previous_input;
    double _integration_term;
};

#endif
