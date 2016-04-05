#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>

// Uncomment for debugging
//#define DEBUG_ENABLED

// Uncomment depending on what type of output is desired (quaternion, euler, yaw pitch roll)
//#define OUTPUT_QUATERNION
//#define OUTPUT_EULER
#define OUTPUT_YAWPITCHROLL

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
  // Begin I2C and Serial communications
  Wire.begin();
  Serial.begin(115200);

  // Initialize MPU
  #ifdef DEBUG_ENABLED
    Serial.println("Initializing the MPU...");
  #endif
  mpu.initialize();

  #ifdef DEBUG_ENABLED
    // Test connection
    if(mpu.testConnection())
    {
      Serial.println("MPU connected.");
    }
    else
    {
      Serial.println("Connection to MPU failed.");
    }
  #endif

  // Initialize the DMP
  deviceStatus = mpu.dmpInitialize();

  if(deviceStatus == 0)
  {
    // Everything went well
    #ifdef DEBUG_ENABLED
      Serial.println("Enabling DMP...");
    #endif
    mpu.setDMPEnabled(true);

    // Attach interrupt
    #ifdef DEBUG_ENABLED
      Serial.println("Enabling external interrupt detection...");
    #endif
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // Set DMP ready flag
    #ifdef DEBUG_ENABLED
      Serial.println("DMP ready. Listening for interrupt...");
    #endif
    dmpReady = true;

    // Set expected packet size
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    #ifdef DEBUG_ENABLED
      // Error (1 = memory load failed, 2 = DMP configuration update failed)
      Serial.print("DMP initialization failed with error code ");
      Serial.print(deviceStatus);
      Serial.println(".");
    #endif
  }
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
  }
  else if(mpuIntStatus & 0x02)
  {
    // Wait to fill FIFO packet
    while(fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // Packet is ready, read it now
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // Allows for immediate reading of another available packet without an interrupt
    fifoCount -= packetSize;

    // Output data
    #ifdef OUTPUT_QUATERNION
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      Serial.println(String(q.w) + " " + String(q.x) + " " + String(q.y) + " " + String(q.z));
    #endif

    #ifdef OUTPUT_EULER
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetEuler(euler, &q);
      Serial.println(String(euler[0] * 180/M_PI) + " " + String(euler[1] * 180/M_PI) + " " + String(euler[2] * 180/M_PI));
    #endif

    #ifdef OUTPUT_YAWPITCHROLL
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      Serial.println(String(ypr[0] * 180/M_PI) + " " + String(ypr[1] * 180/M_PI) + " " + String(ypr[2] * 180/M_PI));
    #endif

    delay(15);
  }
}
