#include "Config.h"
#include <Servo.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>

Servo left_esc, right_esc;

#define START_BTN_PIN   8
#define LEFT_ESC_PIN    3
#define RIGHT_ESC_PIN   9

int left_esc_value = 700;
int right_esc_value = 700;

int throttle = 900;
float input = 0;
float output = 0;
//PID pitch_pid(&input, &output, 0, 1.8, 0.04, 60, -200, 200, 0.001); // Current ideal: 2.3 0.07 124

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
void dmpDataReady() 
{
    mpuInterrupt = true;
}

void setup() 
{
  // Begin I2C communication
  Wire.begin();
  TWBR = 24; // set 400kHz mode @ 16MHz CPU or 200kHz mode @ 8MHz CPU

  // Begin serial communication
  Serial.begin(115200);

  // Initialize MPU
  Serial.println("Initializing the MPU...");
  mpu.initialize();

  // Test connection
  if(mpu.testConnection())
  {
    Serial.println("MPU connected.");
  }
  else
  {
    Serial.println("Connection to MPU failed.");
  }

  // Initialize the DMP
  deviceStatus = mpu.dmpInitialize();

  if(deviceStatus == 0)
  {
    // Everything went well
    Serial.println("Enabling DMP...");
    mpu.setDMPEnabled(true);

    // Attach interrupt
    Serial.println("Enabling external interrupt detection...");
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // Set DMP ready flag
    Serial.println("DMP ready. Listening for interrupt...");

    dmpReady = true;

    // Set expected packet size
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    // Error (1 = memory load failed, 2 = DMP configuration update failed)
    Serial.print("DMP initialization failed with error code ");
    Serial.print(deviceStatus);
    Serial.println(".");
  }

  /*
  // Setup PID related things
  left_esc.attach(LEFT_ESC_PIN);
  right_esc.attach(RIGHT_ESC_PIN);

  left_esc.writeMicroseconds(700);
  right_esc.writeMicroseconds(700); 

  // Wait for push button to begin program
  Serial.println("Press button to start...");
  while(digitalRead(START_BTN_PIN));
  */
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
    Serial.println("FIFO Overflow!");
  }
  else if(mpuIntStatus & 0x02)
  {
    // Wait to fill FIFO packet
    while(fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // Packet is ready, read it now
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // Allows for immediate reading of another available packet without an interrupt
    fifoCount -= packetSize;

    // Process PID
    processPID();
  }
}

void processPID()
{
  // Get MPU data
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  float pitch = ypr[1] * 180/M_PI;

  Serial.println(pitch);

  /*
  // Update the input to the current pitch
  input = pitch;

  // Run calculation
  pitch_pid.calculate();

  //left_esc.writeMicroseconds(throttle - output);
  //right_esc.writeMicroseconds(throttle + output);

  Serial.println("Pitch: " + String(pitch) + ", PID output: " + String(output) + ", Kp: " + String(pitch_pid.getKp())  + ", Ki: " + String(pitch_pid.getKi())  + ", Kd: " + String(pitch_pid.getKd()));

  if(Serial.available() > 0)
  {
    throttle = Serial.parseInt();
    pitch_pid.setSetpoint(Serial.parseInt());
    pitch_pid.setKp(Serial.parseFloat());
    pitch_pid.setKi(Serial.parseFloat());
    pitch_pid.setKd(Serial.parseFloat());
  }
  */
}

