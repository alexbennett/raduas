/**
 * Created by Alex Bennett
 * http://alexben.net/
 *
 * Version: 0.1
 * File created: 04/18/2016
 * License: MIT
 */

#include <Arduino.h>
#include <Receiver.h>

// Define receiver pins and receiver object
Receiver receiver;

// Create prototypes
void printInfo(String);
void printWarning(String);
void printError(String);

void setup()
{
    // Startup serial
    Serial.begin(115200);
}

void loop()
{
    printInfo("Throttle: " + String(receiver.getThrottle()) + ", Yaw: " + String(receiver.getYaw()) + ", Pitch: " + String(receiver.getPitch()) + ", Roll: " + String(receiver.getRoll()));
}

/**
 * Pass PCINT1_vect interrupt to necessary classes
 */
ISR(PCINT2_vect)
{
    receiver.handleInterrupt();
}

/**
 * Prints the given message to the console with "[INFO]"
 * appended to the beginning.
 * @param msg the message to print.
 */
void printInfo(String msg)
{
    Serial.println("[INFO] " + msg);
}

/**
 * Prints the given message to the console with "[WARNING]"
 * appended to the beginning.
 * @param msg the message to print.
 */
void printWarning(String msg)
{
    Serial.println("[WARNING] " + msg);
}

/**
 * Prints the given message to the console with "[ERROR]"
 * appended to the beginning.
 * @param msg the message to print.
 */
void printError(String msg)
{
    Serial.println("[ERROR] " + msg);
}
