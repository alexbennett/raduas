/**
 * Created by Alex Bennett
 * http://alexben.net/
 *
 * Version: 0.1
 * File created: 04/18/2016
 * License: MIT
 *
 * Description: This library is used to read the values of the AR610 receiver
 *              used for controlling the quadcopter. It works on pins 8, 9, 10,
 *              and 11 only.
 *
 *              The pin mapping is defined as the following:
 *              A0 -> Throttle
 *              A1 -> Yaw
 *              A2 -> Pitch
 *              A3 -> Roll
 */

#ifndef RECEIVER_H
#define RECEIVER_H

#include <Arduino.h>

class Receiver
{
public:
    Receiver();

    void handleInterrupt();

    int getThrottle();
    int getYaw();
    int getPitch();
    int getRoll();
private:
    volatile uint16_t _throttle_value, _yaw_value, _pitch_value, _roll_value;
    volatile uint8_t _throttle_status, _yaw_status, _pitch_status, _roll_status;
    volatile uint16_t _throttle_timer, _yaw_timer, _pitch_timer, _roll_timer;
};

#endif
