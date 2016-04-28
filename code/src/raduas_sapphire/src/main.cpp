/**
 * Created by Alex Bennett
 * http://alexben.net/
 *
 * Version: 0.1
 * File created: 04/18/2016
 * License: MIT
 */

#include <Arduino.h>
#include <PID.h>
#include <Servo.h>
#include <Receiver.h>

// Define receiver pins and receiver object
#define THROTTLE_PIN 10
#define YAW_PIN 11
#define PITCH_PIN 12
#define ROLL_PIN 13

Receiver receiver(THROTTLE_PIN, YAW_PIN, PITCH_PIN, ROLL_PIN);

int throttle_value = 0;
int yaw_value = 0;
int pitch_value = 0;
int roll_value = 0;

#define MIN_THROTTLE_POSITION 1000

// Define PID constants
#define YAW_PID_KP 0.5
#define YAW_PID_KI 1.1
#define YAW_PID_KD 1.0
#define YAW_PID_INITIAL_SETPOINT 0
#define PITCH_PID_KP 0.5
#define PITCH_PID_KI 1.1
#define PITCH_PID_KD 1.6
#define PITCH_PID_INITIAL_SETPOINT 0
#define ROLL_PID_KP 0.5
#define ROLL_PID_KI 1.1
#define ROLL_PID_KD 1.6
#define ROLL_PID_INITIAL_SETPOINT 0

// Define PID-related variables
int update_frequency = 1000; // Hz
double yaw_input = 0;
double yaw_output = 0;
double pitch_input = 0;
double pitch_output = 0;
double roll_input = 0;
double roll_output = 0;

// Create PIDs
PID yaw_pid(&yaw_input, &yaw_output, YAW_PID_INITIAL_SETPOINT, YAW_PID_KP, YAW_PID_KI, YAW_PID_KD, 0, 1000);
PID pitch_pid(&pitch_input, &pitch_output, PITCH_PID_INITIAL_SETPOINT, PITCH_PID_KP, PITCH_PID_KI, PITCH_PID_KD, 0, 1000);
PID roll_pid(&roll_input, &roll_output, ROLL_PID_INITIAL_SETPOINT, ROLL_PID_KP, ROLL_PID_KI, ROLL_PID_KD, 0, 1000);

// Setup ESCs and speeds
#define FRONT_MOTOR_PIN 3
#define REAR_MOTOR_PIN 5
#define LEFT_MOTOR_PIN 6
#define RIGHT_MOTOR_PIN 9

Servo front_motor, rear_motor, left_motor, right_motor;

ISR(TIMER2_COMPA_vect)
{
    yaw_pid.update();
    pitch_pid.update();
    roll_pid.update();
}

void setup()
{
    // Startup serial
    Serial.begin(115200);

    // Setup receiver values
    receiver.setChannelPtr(1, &throttle_value);
    receiver.setChannelPtr(2, &yaw_value);
    receiver.setChannelPtr(3, &pitch_value);
    receiver.setChannelPtr(4, &roll_value);

    // Attach ESCs and set start speed
    front_motor.attach(FRONT_MOTOR_PIN);
    rear_motor.attach(REAR_MOTOR_PIN);
    left_motor.attach(LEFT_MOTOR_PIN);
    right_motor.attach(RIGHT_MOTOR_PIN);

    front_motor.writeMicroseconds(MIN_THROTTLE_POSITION);
    rear_motor.writeMicroseconds(MIN_THROTTLE_POSITION);
    left_motor.writeMicroseconds(MIN_THROTTLE_POSITION);
    right_motor.writeMicroseconds(MIN_THROTTLE_POSITION);

    // Setup update timer interrupt
    noInterrupts();
    TCCR2A = 0;
	TCCR2B = 0;
	TCCR2B |= (1 << WGM22);
	TCCR2B |= (1 << CS22 | 1 << CS20);
    TCNT2 = 0;
	OCR2A = 15625 / update_frequency; // (clockFreq) / (desiredFreq * preScalar) - 1 (must be <65536)
	TIMSK2 |= (1 << OCIE2A);
    interrupts();

    pinMode(THROTTLE_PIN, INPUT);
}

void loop()
{
    // Update values from transmitter
    receiver.update();

    if(throttle_value < 1100)
    {
        front_motor.writeMicroseconds(MIN_THROTTLE_POSITION);
        rear_motor.writeMicroseconds(MIN_THROTTLE_POSITION);
        left_motor.writeMicroseconds(MIN_THROTTLE_POSITION);
        right_motor.writeMicroseconds(MIN_THROTTLE_POSITION);
    }
    else
    {
        front_motor.writeMicroseconds(throttle_value);
        rear_motor.writeMicroseconds(throttle_value);
        left_motor.writeMicroseconds(throttle_value);
        right_motor.writeMicroseconds(throttle_value);
    }

    Serial.println("Throttle: " + String(throttle_value) + ", Yaw: " + String(yaw_value) + ", Pitch: " + String(pitch_value) + ", Roll: " + String(roll_value));

    delay(50);
}
