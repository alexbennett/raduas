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
#define YAW_PID_KP 0
#define YAW_PID_KI 0
#define YAW_PID_KD 0
#define YAW_PID_INITIAL_SETPOINT 0

#define PITCH_PID_KP 5.2
#define PITCH_PID_KI 0.01
#define PITCH_PID_KD 0
#define PITCH_PID_INITIAL_SETPOINT 0

#define ROLL_PID_KP 5.2
#define ROLL_PID_KI 0.01
#define ROLL_PID_KD 0
#define ROLL_PID_INITIAL_SETPOINT 0

// Define PID-related variables
double yaw_input = 0;
double yaw_output = 0;
double pitch_input = 0;
double pitch_output = 0;
double roll_input = 0;
double roll_output = 0;

// Create PIDs
PID yaw_pid(&yaw_input, &yaw_output, YAW_PID_INITIAL_SETPOINT, YAW_PID_KP, YAW_PID_KI, YAW_PID_KD, 0, 200, 50);
PID pitch_pid(&pitch_input, &pitch_output, PITCH_PID_INITIAL_SETPOINT, PITCH_PID_KP, PITCH_PID_KI, PITCH_PID_KD, 0, 200, 50);
PID roll_pid(&roll_input, &roll_output, ROLL_PID_INITIAL_SETPOINT, ROLL_PID_KP, ROLL_PID_KI, ROLL_PID_KD, 0, 200, 50);

// Setup ESCs and speeds
#define FRONT_MOTOR_PIN 8
#define REAR_MOTOR_PIN 9
#define LEFT_MOTOR_PIN 7
#define RIGHT_MOTOR_PIN 6

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
void calibrateIMU();
void calculatePID();
void updateYPRInputs();
void updateMotors();
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
     ************* Setup IMU *************
     *************************************/
    // Begin I2C communication
    Wire.begin();
    Wire.setClock(400000);

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

        // Run calibration to set offsets
        //calibrateIMU();
        mpu.setXAccelOffset(-5756);
        mpu.setYAccelOffset(-309);
        mpu.setZAccelOffset(1511);
        mpu.setXGyroOffset(90);
        mpu.setYGyroOffset(-11);
        mpu.setZGyroOffset(-32);
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

    printInfo("Waiting for arming signal...");
    while(receiver.getThrottle() != 0 && (receiver.getThrottle() > 1100 || receiver.getYaw() > 1200));
    printInfo("System armed.");
}

void loop()
{
    // Return until the DMP is ready
    if(dmpReady)
    {
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
            updateYPRInputs();
        }
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
        uint16_t throttle = receiver.getThrottle();
        uint16_t fl_motor_speed, fr_motor_speed, bl_motor_speed, br_motor_speed;

        // Limit max throttle to allow room for PID adjustment
        if(throttle > 1800) throttle = 1800;

        // Perform kinematic calculations
        fl_motor_speed = throttle + pitch_output;
        fr_motor_speed = throttle - roll_output;
        bl_motor_speed = throttle + roll_output;
        br_motor_speed = throttle - pitch_output;

        // Don't let the PID dip below the minimum ESC pulse length
        if(fl_motor_speed < 1200) fl_motor_speed = 1200;
        if(fr_motor_speed < 1200) fr_motor_speed = 1200;
        if(bl_motor_speed < 1200) bl_motor_speed = 1200;
        if(br_motor_speed < 1200) br_motor_speed = 1200;

        // Don't let the PID rise above the maximum ESC pulse length
        if(fl_motor_speed > 2000) fl_motor_speed = 2000;
        if(fr_motor_speed > 2000) fr_motor_speed = 2000;
        if(bl_motor_speed > 2000) bl_motor_speed = 2000;
        if(br_motor_speed > 2000) br_motor_speed = 2000;

        front_motor.writeMicroseconds(fl_motor_speed);
        right_motor.writeMicroseconds(fr_motor_speed);
        left_motor.writeMicroseconds(bl_motor_speed);
        rear_motor.writeMicroseconds(br_motor_speed);

        printInfo("Throttle: " + String(receiver.getThrottle()) + ", FL: " + String(fl_motor_speed) + ", FR: " + String(fr_motor_speed) + ", BL: " + String(bl_motor_speed) + ", BR: " + String(br_motor_speed));
    }

    // Calculate PID
    calculatePID();
}

/**
 * Pass PCINT1_vect interrupt to necessary classes
 */
ISR(PCINT2_vect)
{
    receiver.handleInterrupt();
}

/**
 * Calibrates the IMU offsets
 */
void calibrateIMU()
{
    // Set offsets
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    mpu.setZAccelOffset(0);
}

/**
 * Run PID calculation updates
 */
void calculatePID()
{
    yaw_pid.update();
    pitch_pid.update();
    roll_pid.update();
}

/**
 * Updates the inputs to PID
 */
void updateYPRInputs()
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
