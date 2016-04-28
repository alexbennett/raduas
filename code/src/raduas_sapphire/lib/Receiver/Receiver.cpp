/**
 * Created by Alex Bennett
 * http://alexben.net/
 *
 * Version: 0.1
 * File created: 04/18/2016
 * License: MIT
 */

#include "Receiver.h"

Receiver::Receiver(uint8_t ch1_pin, uint8_t ch2_pin, uint8_t ch3_pin, uint8_t ch4_pin)
{
    this->_ch1_pin = ch1_pin;
    this->_ch2_pin = ch2_pin;
    this->_ch3_pin = ch3_pin;
    this->_ch4_pin = ch4_pin;

    pinMode(this->_ch1_pin, INPUT);
    pinMode(this->_ch2_pin, INPUT);
    pinMode(this->_ch3_pin, INPUT);
    pinMode(this->_ch4_pin, INPUT);
}

void Receiver::update()
{
    *(this->_ch1_ptr) = pulseIn(this->_ch1_pin, HIGH);
    *(this->_ch2_ptr) = pulseIn(this->_ch2_pin, HIGH);
    *(this->_ch3_ptr) = pulseIn(this->_ch3_pin, HIGH);
    *(this->_ch4_ptr) = pulseIn(this->_ch4_pin, HIGH);
}

void Receiver::setMinValue(uint8_t min_value)
{

}

void Receiver::setMaxValue(uint8_t max_value)
{

}

void Receiver::setChannelPtr(uint8_t channel, int* ptr)
{
    switch(channel)
    {
        case 1:
            this->_ch1_ptr = ptr;
            break;
        case 2:
            this->_ch2_ptr = ptr;
            break;
        case 3:
            this->_ch3_ptr = ptr;
            break;
        case 4:
            this->_ch4_ptr = ptr;
            break;
    }
}
