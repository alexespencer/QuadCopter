// ================================================================
// ===             SETUP for Reading the MPU6050 sensor         ===
// ===             + other routines for stabilisation           ===
// ================================================================

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

//Class for the MPU
MPU6050 mpu;

//Store when the DMP was started
unsigned long DMPStartedMillis = 0;

//Send a serial debug message every x millis
long serialDebug = 250;
unsigned long lastSerialDebug = 0;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// ================================================================
// ===               ORIENTATION/MOTION VARIABLES               ===
// ================================================================
int16_t gyro[3];       // [x,y,z] Gryo readings (roll, pitch and yaw)
float yrp[3];           // [yaw, roll, pitch]
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector

//Store yaw 1 second ago, so we know when stable (then arm motors)
float yaw_1sec;
unsigned long yaw_millis = 0;
bool yaw_stable = false;

// ================================================================
// ===               SETUP routine - runs once                  ===
// ================================================================
void setup()
{
  Serial.begin(115200);

  // Join I2C bus
  Wire.begin();

  // Initialize MPU6050
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // Verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // Make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    //Set offset
    //mpu.setZGyroOffset(10);
    //mpu.setZAccelOffset(1788);
    
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;
    DMPStartedMillis = millis();

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } 
  else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() 
{ 
  //Get latest fifoBuffer from MPU
  byte bufferUpdated = GetLatestPacket();

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(yrp, &q, &gravity);
  mpu.dmpGetGyro(gyro, fifoBuffer);

  //Convert yrp into degrees
  yrp[0] = yrp[0] * 180/M_PI;
  yrp[1] = yrp[1] * 180/M_PI;
  yrp[1] = yrp[1];
  yrp[2] = yrp[2] * 180/M_PI;
  yrp[2] = yrp[2];

  //Wrap around yaw
  if (yrp[0] < -180) {
    yrp[0]+360;
  }
  else if (yrp[0] > 180) {
    yrp[0]-360;
  }
  
  // ================================================================
  // ===       Detect stable yaw (when perc change is small)      ===
  // ================================================================

  if (yaw_stable == false && millis() > 2000 && millis() - yaw_millis > 1000 )
  {
    yaw_millis = millis();
    
    //Compare yaw now to yaw 1 second ago
    if (abs(yrp[0] - yaw_1sec) > 0.1)
    {
     //Yaw unstable - wait, but record this current yaw
     yaw_1sec = yrp[0]; 
    }
    else
    {
      //Yaw is stable, can now fly/arm motors as required
      yaw_stable = true;
    }
  }

  // ================================================================
  // ===             Debug section - Write to serial port         ===
  // ================================================================

  if (millis() - lastSerialDebug > serialDebug) {
    lastSerialDebug = millis();
    
    Serial.print("Time (ms):\t");
    Serial.print(millis());
    Serial.print("\tYaw:\t");
    Serial.print(yrp[0], 3);
    Serial.print("\tGyro Yaw:\t");
    Serial.print(gyro[2]);
    Serial.print("\tYaw stable:\t");
    Serial.println(yaw_stable);
    
  }
}

// ================================================================
// ===                  Function to get MPU readings            ===
// ================================================================
byte GetLatestPacket()
{
  //Get current FIFO count
  fifoCount = mpu.getFIFOCount();

  mpuIntStatus = mpu.getIntStatus();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    return 0;
  }
  else {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    //Keep reading until there is not a full packet left
    while (fifoCount >= packetSize)
    {
      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);

      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;
    }

    return 1;
  }
}





