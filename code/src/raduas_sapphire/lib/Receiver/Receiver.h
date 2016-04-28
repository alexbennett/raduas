/**
 * Created by Alex Bennett
 * http://alexben.net/
 *
 * Version: 0.1
 * File created: 04/18/2016
 * License: MIT
 */

#ifndef RECEIVER_H
#define RECEIVER_H

#include <Arduino.h>

class Receiver
{
public:
    Receiver(uint8_t, uint8_t, uint8_t, uint8_t);

    void update();
    void setMinValue(uint8_t);
    void setMaxValue(uint8_t);
    void setChannelPtr(uint8_t, int*);

private:
    uint8_t _ch1_pin;
    uint8_t _ch2_pin;
    uint8_t _ch3_pin;
    uint8_t _ch4_pin;

    int* _ch1_ptr;
    int* _ch2_ptr;
    int* _ch3_ptr;
    int* _ch4_ptr;
};

#endif
