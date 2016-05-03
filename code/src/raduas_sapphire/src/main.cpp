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
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>

// Define receiver pins and receiver object
Receiver receiver;

#define MIN_THROTTLE_POSITION 1000
#define THROTTLE_ON_POSITION 1100

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
#define UPDATE_FREQ 1000 // Hz
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
#define FRONT_MOTOR_PIN 6
#define REAR_MOTOR_PIN 7
#define LEFT_MOTOR_PIN 8
#define RIGHT_MOTOR_PIN 9

Servo front_motor, rear_motor, left_motor, right_motor;

// Create MPU object
MPU6050 mpu;

// Control and status variables
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t deviceStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

// Create memory to store orientation and motion readings
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;

// Create prototypes
void updateYPR();
void printInfo(String);
void printWarning(String);
void printError(String);

void dmpDataReady()
{
    mpuInterrupt = true;
}

void setup()
{
    // Startup serial
    Serial.begin(115200);

    // Attach ESCs
    front_motor.attach(FRONT_MOTOR_PIN);
    rear_motor.attach(REAR_MOTOR_PIN);
    left_motor.attach(LEFT_MOTOR_PIN);
    right_motor.attach(RIGHT_MOTOR_PIN);

    // Set initial start speed to minimum throttle position
    front_motor.writeMicroseconds(MIN_THROTTLE_POSITION);
    rear_motor.writeMicroseconds(MIN_THROTTLE_POSITION);
    left_motor.writeMicroseconds(MIN_THROTTLE_POSITION);
    right_motor.writeMicroseconds(MIN_THROTTLE_POSITION);

    /*************************************
     *** Setup update timer interrupts ***
     *************************************/
    // Disable interrupts
    cli();
    // Reset TCCR2 registers to 0
    TCCR2A = 0;
	TCCR2B = 0;

    // Set compare registers for TIMER2
    OCR2A = 15625 / UPDATE_FREQ; // clock_frequency / desired_frequency

    // Enable clear timer mode (CTC)
	TCCR2B |= (1 << WGM22);

    // Set prescale to 1024
    TCCR2B |= (1 << CS20);
	TCCR2B |= (1 << CS22);

    // Finally, enable the compare interrupt
	TIMSK2 |= (1 << OCIE2A);

    // Re-enable interrupts
    sei();
    /*********************************
     *** End timer interrupt setup ***
     *********************************/

    /*************************************
     ************* Setup IMU *************
     *************************************/
    // Begin I2C communication
    Wire.begin();
    TWBR = 12;
    //Wire.setClock(400000);

    // Initialize MPU
    printInfo("Initializing the MPU...");
    mpu.initialize();

    // Test connection
    if(mpu.testConnection())
    {
        printInfo("MPU connected.");
    }
    else
    {
        printError("Connection to MPU failed.");
    }

    // Initialize the DMP
    deviceStatus = mpu.dmpInitialize();

    // Set offsets
    //mpu.setXGyroOffset(50);
    //mpu.setYGyroOffset(0);
    //mpu.setZGyroOffset(0);
    //mpu.setZAccelOffset(0);

    // Check IMU status and react accordingly
    if(deviceStatus == 0)
    {
        // Everything went well
        printInfo("Enabling DMP...");
        mpu.setDMPEnabled(true);

        // Attach interrupt
        printInfo("Enabling external interrupt detection...");
        attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // Set DMP ready flag
        printInfo("DMP ready. Listening for interrupt...");

        dmpReady = true;

        // Set expected packet size
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
        // Error (1 = memory load failed, 2 = DMP configuration update failed)
        printError("DMP initialization failed with error code: ");
        printError(String(deviceStatus));
    }
    /*************************************
     *********** End IMU setup ***********
     *************************************/

    // TODO: Wait for user to arm the device (throttle off and to the right)
    printInfo("Waiting for arming signal...");
    while(receiver.getThrottle() != 0 && (receiver.getThrottle() > 1100 || receiver.getYaw() > 1200));

    printInfo("System armed.");
}

void loop()
{
    // Return until the DMP is ready
    if(!dmpReady) return;

    // Reset interrupt status flag
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // Get FIFO count
    fifoCount = mpu.getFIFOCount();

    // Check for overflow just in case
    if((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // Reset FIFO
        mpu.resetFIFO();
        printWarning("FIFO overflow");
    }
    else if(mpuIntStatus & 0x02)
    {
        // Wait to fill FIFO packet
        while(fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // Packet is ready, read it now
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        mpu.resetFIFO();

        // Allows for immediate reading of another available packet without an interrupt
        fifoCount -= packetSize;

        // Update yaw, pitch, and roll for the PID
        updateYPR();
    }

    if(receiver.getThrottle() < THROTTLE_ON_POSITION)
    {
        // Immediately turn off motors
        front_motor.writeMicroseconds(MIN_THROTTLE_POSITION);
        rear_motor.writeMicroseconds(MIN_THROTTLE_POSITION);
        left_motor.writeMicroseconds(MIN_THROTTLE_POSITION);
        right_motor.writeMicroseconds(MIN_THROTTLE_POSITION);
    }
    else
    {
        //front_motor.writeMicroseconds(receiver.getThrottle());
        //rear_motor.writeMicroseconds(receiver.getThrottle());

        int output1 = 1200 - pitch_output;
        int output2 = 1200 + pitch_output;

        left_motor.writeMicroseconds(output1);
        right_motor.writeMicroseconds(output2);

        printInfo("Throttle: " + String(receiver.getThrottle()) + ", Output 1: " + String(output1) + ", Output 2: " + String(output2));
    }

}

/**
 * Pass PCINT1_vect interrupt to necessary classes
 */
ISR(PCINT2_vect)
{
    receiver.handleInterrupt();
}

/**
 * Update PIDs based on overflow of TIMER2
 */
ISR(TIMER2_COMPA_vect)
{
    yaw_pid.update();
    pitch_pid.update();
    roll_pid.update();
}

/**
 * Updates the inputs to PID
 */
void updateYPR()
{
    // Grab yaw, pitch, and roll
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // Update yaw, pitch, and roll input
    yaw_input = ypr[0] * 180/M_PI;
    pitch_input = ypr[1] * 180/M_PI;
    roll_input = ypr[2] * 180/M_PI;
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
