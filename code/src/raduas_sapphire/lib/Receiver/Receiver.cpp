/**
 * Created by Alex Bennett
 * http://alexben.net/
 *
 * Version: 0.1
 * File created: 04/18/2016
 * License: MIT
 */

#include "Receiver.h"

Receiver::Receiver()
{
    // Set initial values
    this->_throttle_value = 0;
    this->_throttle_status = 0;
    this->_throttle_timer = 0;
    this->_yaw_value = 0;
    this->_yaw_status = 0;
    this->_yaw_timer = 0;
    this->_pitch_value = 0;
    this->_pitch_status = 0;
    this->_pitch_timer = 0;
    this->_roll_value = 0;
    this->_roll_status = 0;
    this->_roll_timer = 0;

    // Set pins to inputs with pull-up resistors
    pinMode(A8, INPUT_PULLUP);
    pinMode(A9, INPUT_PULLUP);
    pinMode(A10, INPUT_PULLUP);
    pinMode(A11, INPUT_PULLUP);

    // Setup pin change interrupts
    PCICR |= (1 << PCIE2); // Enable PCMSK2
    PCMSK2 |= (1 << PCINT16); // Enable PCINT16 (A8)
    PCMSK2 |= (1 << PCINT17); // Enable PCINT17 (A9)
    PCMSK2 |= (1 << PCINT18); // Enable PCINT18 (A10)
    PCMSK2 |= (1 << PCINT19); // Enable PCINT19 (A11)
}

void Receiver::handleInterrupt()
{
    // Read throttle
    if(this->_throttle_status == 0 && (PINK & 0x01)) // Pin rose
    {
        // Set status to HIGH
        this->_throttle_status = 1;

        // Start timing
        this->_throttle_timer = micros();
    }
    else if(this->_throttle_status == 1 && !(PINK & 0x01)) // Pin fell
    {
        // Set status to LOW
        this->_throttle_status = 0;

        // Stop timing and set the throttle value
        this->_throttle_value = micros() - this->_throttle_timer;
    }

    // Read yaw
    if(!this->_yaw_status && (PINK & 0x02)) // Pin rose
    {
        // Set status to HIGH
        this->_yaw_status = 1;

        // Start timing
        this->_yaw_timer = micros();
    }
    else if(this->_yaw_status && !(PINK & 0x02)) // Pin fell
    {
        // Set status to LOW
        this->_yaw_status = 0;

        // Stop timing and set the throttle value
        this->_yaw_value = micros() - this->_yaw_timer;
    }

    // Read pitch
    if(!this->_pitch_status && (PINK & 0x04)) // Pin rose
    {
        // Set status to HIGH
        this->_pitch_status = 1;

        // Start timing
        this->_pitch_timer = micros();
    }
    else if(this->_pitch_status && !(PINK & 0x04)) // Pin fell
    {
        // Set status to LOW
        this->_pitch_status = 0;

        // Stop timing and set the throttle value
        this->_pitch_value = micros() - this->_pitch_timer;
    }

    // Read roll
    if(!this->_roll_status && (PINK & 0x08)) // Pin rose
    {
        // Set status to HIGH
        this->_roll_status = 1;

        // Start timing
        this->_roll_timer = micros();
    }
    else if(this->_roll_status && !(PINK & 0x08)) // Pin fell
    {
        // Set status to LOW
        this->_roll_status = 0;

        // Stop timing and set the throttle value
        this->_roll_value = micros() - this->_roll_timer;
    }
}

int Receiver::getThrottle()
{
    return this->_throttle_value;
}

int Receiver::getYaw()
{
    return this->_yaw_value;
}

int Receiver::getPitch()
{
    return this->_pitch_value;
}

int Receiver::getRoll()
{
    return this->_roll_value;
}
