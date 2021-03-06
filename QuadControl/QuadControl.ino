/*
Author: Alex Spencer
 Date: 07/04/2014
 
 With huge thanks to:
 -Jeff Rowberg for all his work on the MPU6050 sensor:
 ==========================================================
 I2Cdev device library code is placed under the MIT license
 Copyright (c) 2012 Jeff Rowberg
 
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 ==========================================================
 
 With thanks to:
 -RCArduino - for their article below for reading multiple channels
 http://rcarduino.blogspot.co.uk/2012/04/how-to-read-multiple-rc-channels-draft.html
 
 ==================
 Pin Configuration:
 MPU6050
 SCL to Pin A5
 SDA to Pin A4
 ADO to GROUND - This ensures the address on the I2C bus is 0x68
 
 Motors (ESCs):
 FrontLeft, as defined below
 FrontRight, as defined below
 BackRight, as defined below
 BackLeft, as defined below
 */

// ================================================================
// ===             SETUP for Reading Radio Channels             ===
// ================================================================
#include <PinChangeInt.h>

// Assign your channel in pins
#define THROTTLE_IN_PIN 3
#define PITCH_IN_PIN 4
#define ROLL_IN_PIN 2
#define YAW_IN_PIN 5
#define PT_IN_PIN 7
#define HT_IN_PIN 8
#define HP_IN_PIN 12
#define AUX_IN_PIN 13

// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals
#define THROTTLE_FLAG 1
#define PITCH_FLAG 2
#define ROLL_FLAG 4
#define YAW_FLAG 8
#define PT_FLAG 16
#define HT_FLAG 32
#define HP_FLAG 64
#define AUX_FLAG 128

// holds the update flags defined above
volatile uint8_t bUpdateFlagsShared;

// shared variables are updated by the ISR and read by loop.
volatile uint16_t unThrottleInShared;
volatile uint16_t unPitchInShared;
volatile uint16_t unRollInShared;
volatile uint16_t unYawInShared;
volatile uint16_t unPTInShared;
volatile uint16_t unHTInShared;
volatile uint16_t unHPInShared;
volatile uint16_t unAuxInShared;

// These are used to record the rising edge of a pulse in the calcInput functions
// They do not need to be volatile as they are only used in the ISR. If we wanted
// to refer to these in loop and the ISR then they would need to be declared volatile
uint32_t ulThrottleStart;
uint32_t ulPitchStart;
uint32_t ulRollStart;
uint32_t ulYawStart;
uint32_t ulPTStart;
uint32_t ulHTStart;
uint32_t ulHPStart;
uint32_t ulAuxStart;

//Min/Max ranges known for each channel
#define MINRC_ROLL 1040
#define MINRC_PITCH 1052
#define MINRC_YAW 1056
#define MINRC_THROTTLE 1048
#define MINRC_PT 1904
#define MINRC_AUX 1028
#define MINRC_HT 1060
#define MINRC_HP 1052

#define MAXRC_ROLL 1872
#define MAXRC_PITCH 1892
#define MAXRC_YAW 1892
#define MAXRC_THROTTLE 1884
#define MAXRC_PT 1036
#define MAXRC_AUX 1888
#define MAXRC_HT 1888
#define MAXRC_HP 1876

// ================================================================
// ===             SETUP for Reading the MPU6050 sensor         ===
// ===             + other routines for stabilisation           ===
// ================================================================

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "PID_v1.h"
#include "Servo.h"
#include "MemoryFree.h"
#include "EEPROMex.h"

//Class for the MPU
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

//Store when the DMP was started
unsigned long DMPStartedMillis = 0;

// ================================================================
// ===            Variables for Mode of Quad (Acro/Stab)        ===
// ================================================================

#define ACRO_MODE 1
#define STAB_MODE 2

byte quadMode = 1;
byte lastQuadMode = 0;

long esc_FL = 700, esc_FR = 700, esc_BL = 700, esc_BR = 700;

// ================================================================
// ===               Pin Settings for Motors                    ===
// ================================================================
#define MOTOR_FL 6 //Pin D6 (PWM)
#define MOTOR_FR 9 //Pin D9 (PWM)
#define MOTOR_BL 10 //Pin D10 (PWM)
#define MOTOR_BR 11 //Pin D11 (PWM)

Servo motorFL;
Servo motorFR;
Servo motorBL;
Servo motorBR;

// ================================================================
// ===               Parameters for Min Max throttle, etc       ===
// ================================================================

//What throttle value will start the motors? Need room for movement for balancing
#define MIN_ESC 850
#define MIN_THROTTLE 900 //This needs to be ABOVE MIN esc, to allow for balancing to happen
#define MAX_THROTTLE 1300 //This needs to be BELOW Max ESC, to allow some room for stabilising. Default: 1700
#define MAX_ESC 1600  //To-do - set this to 2000 - release the potential!

#define RC_MAX_ROLL_RATE 200
#define MAX_ROLL_RATE 400

// ================================================================
// ===               Benchmarking/Debug settings                ===
// ================================================================

//Send a serial debug message every x millis
long serialDebug = 250;
unsigned long lastSerialDebug = 0;

//Benchmarking vars
long benchmark = 0;
long benchmarkResult = 0;
unsigned long lastbenchmark = 0;

// ================================================================
// ===               ORIENTATION/MOTION VARIABLES               ===
// ================================================================
int16_t gyro[3];       // [x,y,z] Gryo readings (roll, pitch and yaw)
int16_t m1_gyro[3];    //Stores the last gyro readings for pitch and roll

float yrp[3];           // [yaw, roll, pitch]
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector

// ================================================================
// ===             Vars to store what "FLAT" is to EEPROM       ===
// ================================================================

bool requestStart = false;
unsigned long requestStartMillis = 0;
bool valuesSaved = false;
bool saveWhenStable = false;

float pitchOffset = -1.79; //If the sensor is on a wonk - this corrects the actual angle read. Overwritten by EEPROM
float rollOffset = 4.19; //If the sensor is on a wonk - this corrects the actual angle read. Overwritten by EEPROM

//Store yaw 1 second ago, so we know when stable (then arm motors)
float yaw_1sec;
unsigned long yaw_millis = 0;
bool yaw_stable = false;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
//Store the mapped throttle being requested by the transmitter
uint16_t rc_throttle;

//Map yaw pitch and roll RC inputs (PWM) into a desired angle
//These are also the set points for the pids
float req_yaw, req_pitch, req_roll;

float yaw_target = 0;

//Class for the PIDs
#define PID_PITCH_RATE 0
#define PID_ROLL_RATE 1
#define PID_PITCH_STAB 2
#define PID_ROLL_STAB 3
#define PID_YAW_RATE 4
#define PID_YAW_STAB 5

//These are the Inputs for the PIDs (what is actually happening)
double pidInput[6];

//These are the outputs for the PIDs (what is the PID controlling?)
double pidOutput[6];

//These are the desired values that you would LIKE the input to be
double pidSetPoint[6];

double rollAndPitchRate_P = 1.27;
double rollAndPitchRate_I = 4.975;
double rollAndPitchRate_D = 0.067625;

double rollAndPitchStab_P = 2.45;
double rollAndPitchStab_I = 0.0;
double rollAndPitchStab_D = 0.0;

double YawRate_P = 0.5;
double YawRate_I = 0.0;
double YawRate_D = 0.0;

double YawStab_P = 1.0;
double YawStab_I = 0.0;
double YawStab_D = 0.0;

PID pid_PITCH_RATE(&pidInput[PID_PITCH_RATE], &pidOutput[PID_PITCH_RATE], &pidSetPoint[PID_PITCH_RATE], rollAndPitchRate_P, rollAndPitchRate_I, rollAndPitchRate_D, DIRECT);
PID pid_ROLL_RATE(&pidInput[PID_ROLL_RATE], &pidOutput[PID_ROLL_RATE], &pidSetPoint[PID_ROLL_RATE], rollAndPitchRate_P, rollAndPitchRate_I, rollAndPitchRate_D, DIRECT);

PID pid_PITCH_STAB(&pidInput[PID_PITCH_STAB], &pidOutput[PID_PITCH_STAB], &pidSetPoint[PID_PITCH_STAB], rollAndPitchStab_P, rollAndPitchStab_I, rollAndPitchStab_D, DIRECT);
PID pid_ROLL_STAB(&pidInput[PID_ROLL_STAB], &pidOutput[PID_ROLL_STAB], &pidSetPoint[PID_ROLL_STAB], rollAndPitchStab_P, rollAndPitchStab_I, rollAndPitchStab_D, DIRECT);

PID pid_YAW_RATE(&pidInput[PID_YAW_RATE], &pidOutput[PID_YAW_RATE], &pidSetPoint[PID_YAW_RATE], YawRate_P, YawRate_I, YawRate_D, DIRECT);
PID pid_YAW_STAB(&pidInput[PID_YAW_STAB], &pidOutput[PID_YAW_STAB], &pidSetPoint[PID_YAW_STAB], YawStab_P, YawStab_I, YawStab_D, DIRECT);

// ================================================================
// ===               SETUP routine - runs once                  ===
// ================================================================
void setup()
{
  //Prepare motors (arm)
  motorFL.attach(MOTOR_FL);
  motorFR.attach(MOTOR_FR); 
  motorBL.attach(MOTOR_BL); 
  motorBR.attach(MOTOR_BR);
  motorFL.writeMicroseconds(700);
  motorFR.writeMicroseconds(700);
  motorBL.writeMicroseconds(700);
  motorBR.writeMicroseconds(700);

  //Start serial bus
  Serial.begin(115200);

  //Set output limits of the PIDs
  pid_PITCH_RATE.SetOutputLimits(-400, 400); //What adjustments can the Rate PIDs sent to Motor ESCs
  pid_ROLL_RATE.SetOutputLimits(-400, 400); //What adjustments can the Rate PIDs sent to Motor ESCs
  pid_YAW_RATE.SetOutputLimits(-300, 300); //What adjustments can the Rate PIDs sent to Motor ESCs

  pid_PITCH_STAB.SetOutputLimits(-MAX_ROLL_RATE, MAX_ROLL_RATE); //What is the min/max requested roll rate from the Stab PID?
  pid_ROLL_STAB.SetOutputLimits(-MAX_ROLL_RATE, MAX_ROLL_RATE); //What is the min/max requested roll rate from the Stab PID?
  pid_YAW_STAB.SetOutputLimits(-360, 360);

  //Limit the timings of the PIDs - this is REALLY important - do not set this BELOW the sample rate of the IMU/MPU6050 - it really causes problems
  pid_PITCH_RATE.SetSampleTime(10);
  pid_ROLL_RATE.SetSampleTime(10);
  pid_YAW_RATE.SetSampleTime(10);
  pid_PITCH_STAB.SetSampleTime(10);
  pid_ROLL_STAB.SetSampleTime(10);
  pid_YAW_STAB.SetSampleTime(10);

  //Read EEPROM to get what "level" is - this obviously needs setting on a "fresh" Arduino board or if the sensor moves
  rollOffset = EEPROM.readFloat(0); //Read current roll (minus this off actual roll to get "LEVEL")
  pitchOffset = EEPROM.readFloat(4); //Read current pitch (minus this off actual pitch to get "LEVEL")
  Serial.println(F("Pitch/Roll offsets read from EEPROM"));

  // using the PinChangeInt library, attach the interrupts
  // used to read the channels
  PCintPort::attachInterrupt(THROTTLE_IN_PIN, calcThrottle, CHANGE); 
  PCintPort::attachInterrupt(PITCH_IN_PIN, calcPitch, CHANGE); 
  PCintPort::attachInterrupt(ROLL_IN_PIN, calcRoll, CHANGE); 
  PCintPort::attachInterrupt(YAW_IN_PIN, calcYaw, CHANGE); 
  PCintPort::attachInterrupt(PT_IN_PIN, calcPT, CHANGE); 
  PCintPort::attachInterrupt(HT_IN_PIN, calcHT, CHANGE); 
  PCintPort::attachInterrupt(HP_IN_PIN, calcHP, CHANGE); 
  PCintPort::attachInterrupt(AUX_IN_PIN, calcAux,CHANGE); 

  //Green LED for YAW stable mode
  pinMode(A0, OUTPUT);
  digitalWrite(A0, LOW);

  // Join I2C bus
  Wire.begin();

  // Initialize MPU6050
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize(); //This sometimes hangs here when using USB - but has never failed when on "battery" power

  // Verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // Make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;
    DMPStartedMillis = millis();

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();

    Serial.println(F("Waiting for Yaw to stabilise"));
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
  //--------Benchmarking----------
  //  benchmark++;
  //
  //  if (millis() - lastbenchmark > 1000)
  //  {
  //    benchmarkResult = benchmark;
  //    benchmark = 0;
  //    lastbenchmark = millis();
  //  }

  // ================================================================
  // ===           Read radio channel inputs                      ===
  // ================================================================ 

  static uint16_t unThrottleIn;
  static uint16_t unPitchIn;
  static uint16_t unRollIn;
  static uint16_t unYawIn;
  static uint16_t unPTIn;
  static uint16_t unHTIn;
  static uint16_t unHPIn;
  static uint16_t unAuxIn;

  // local copy of update flags
  static uint8_t bUpdateFlags;

  // check shared update flags to see if any channels have a new signal
  if(bUpdateFlagsShared)
  {
    noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables

      // take a local copy of which channels were updated in case we need to use this in the rest of loop
    bUpdateFlags = bUpdateFlagsShared;

    // in the current code, the shared values are always populated
    // so we could copy them without testing the flags
    // however in the future this could change, so lets
    // only copy when the flags tell us we can.

    if(bUpdateFlags & THROTTLE_FLAG)
    {
      unThrottleIn = unThrottleInShared;
    }

    if(bUpdateFlags & PITCH_FLAG)
    {
      unPitchIn = unPitchInShared;
    }   

    if(bUpdateFlags & ROLL_FLAG)
    {
      unRollIn = unRollInShared;
    }  

    if(bUpdateFlags & YAW_FLAG)
    {
      unYawIn = unYawInShared;
    }  

    if(bUpdateFlags & PT_FLAG)
    {
      unPTIn = unPTInShared;
    }

    if(bUpdateFlags & HT_FLAG)
    {
      unHTIn = unHTInShared;
    }

    if(bUpdateFlags & PT_FLAG)
    {
      unHPIn = unHPInShared;
    }

    if(bUpdateFlags & AUX_FLAG)
    {
      unAuxIn = unAuxInShared;
    }

    // clear shared copy of updated flags as we have already taken the updates
    // we still have a local copy if we need to use it in bUpdateFlags
    bUpdateFlagsShared = 0;

    interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
  }

  bUpdateFlags = 0;

  //Map the input throttle onto 800-max throttle for more accurate control. No point going above Max throttle
  rc_throttle = map(unThrottleIn, MINRC_THROTTLE, MAXRC_THROTTLE, 800, MAX_THROTTLE);

  // ================================================================
  // ===           Controller Pitch Trim controlls mode           ===
  // ================================================================
  long auxIn = map(unAuxIn, MINRC_AUX, MAXRC_AUX, 0, 1000);

  if (quadMode != ACRO_MODE && auxIn < 400)
  {
    //Mode = ACRO
    quadMode = ACRO_MODE;
  }
  else if (quadMode != STAB_MODE && auxIn > 600)
  {
    //Mode = STAB
    quadMode = STAB_MODE;
  }

  // ================================================================
  // ===           Controller sets P/I/D values of RATE PIDs      ===
  // ================================================================

  //rollAndPitchRate_P = map(unHPIn, MINRC_HP, MAXRC_HP, 0, 1000) / 500.0;
  //rollAndPitchRate_I = map(unPTIn, MINRC_PT, MAXRC_PT, 0, 1000) / 200.0;
  //rollAndPitchRate_D = map(unHTIn, MINRC_HT, MAXRC_HT, 0, 1000) / 8000.0;

  //rollAndPitchRate_P = 0.0;
  //rollAndPitchRate_I = 0.0;

  //Update the PID tunings
  //pid_PITCH_RATE.SetTunings(rollAndPitchRate_P, rollAndPitchRate_I, rollAndPitchRate_D);
  //pid_ROLL_RATE.SetTunings(rollAndPitchRate_P, rollAndPitchRate_I, rollAndPitchRate_D);

  // ================================================================
  // ===           Controller sets P/I values of STAB PIDs        ===
  // ================================================================

  //rollAndPitchStab_P = map(unPTIn, MINRC_PT, MAXRC_PT, 0, 1000) / 400.0;

  //Update the PID tunings
  //pid_PITCH_STAB.SetTunings(rollAndPitchStab_P, rollAndPitchStab_I, rollAndPitchStab_D);
  //pid_ROLL_STAB.SetTunings(rollAndPitchStab_P, rollAndPitchStab_I, rollAndPitchStab_D);

  // ================================================================
  // ===           Controller sets P/I values of YAW PIDs         ===
  // ================================================================

  YawRate_P = map(unHPIn, MINRC_HP, MAXRC_HP, 0, 1000) / 200.0;
  YawRate_I = map(unPTIn, MINRC_PT, MAXRC_PT, 0, 1000) / 2000.0;
  //YawRate_I = 0.0;

  YawStab_P = map(unHTIn, MINRC_HT, MAXRC_HT, 0, 1000) / 100.0;

  //Update the PID tunings
  pid_YAW_RATE.SetTunings(YawRate_P, YawRate_I, YawRate_D);
  pid_YAW_STAB.SetTunings(YawStab_P, YawStab_I, YawStab_D);

  //Keep track of the current gyro readings - used for averaging values later on
  m1_gyro[0] = gyro[0]; //Pitch
  m1_gyro[1] = gyro[1]; //Roll
  m1_gyro[2] = gyro[2]; //Yaw

  //Get latest fifoBuffer from MPU
  byte bufferUpdated = GetLatestPacket();
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(yrp, &q, &gravity);
  mpu.dmpGetGyro(gyro, fifoBuffer);

  //Convert yrp into degrees
  yrp[0] = yrp[0] * 180/M_PI;
  yrp[1] = yrp[1] * 180/M_PI;
  yrp[1] = yrp[1] + rollOffset;
  yrp[2] = yrp[2] * 180/M_PI;
  yrp[2] = yrp[2] + pitchOffset;

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
    else if (rc_throttle < MIN_THROTTLE) //For safety, do not enable Quad if power already up
    {
      //Yaw is stable, can now fly/arm motors as required
      yaw_stable = true;
      digitalWrite(A0, HIGH);

      //Save offsets in EEPROM if requested
      if (saveWhenStable == true)
      {
        saveOffsets();
      }
    }
  }

  //Wrap around yaw
  if (yrp[0] < -180) {
    yrp[0]+360;
  }
  else if (yrp[0] > 180) {
    yrp[0]-360;
  }

  // ================================================================
  // ===                  Power up if Throttle Up                 ===
  // ================================================================

  if (yaw_stable == true && rc_throttle > MIN_THROTTLE)
  {
    //Constrain the throttle
    if (rc_throttle > MAX_THROTTLE) {
      rc_throttle = MAX_THROTTLE;
    }

    //Ensure the PIDS are on
    pid_PITCH_RATE.SetMode(AUTOMATIC);
    pid_ROLL_RATE.SetMode(AUTOMATIC);

    pid_YAW_STAB.SetMode(AUTOMATIC);
    pid_YAW_RATE.SetMode(AUTOMATIC);

    req_yaw = map(unYawIn, MINRC_YAW, MAXRC_YAW, -150, +150);
    req_pitch = map(unPitchIn, MINRC_PITCH, MAXRC_PITCH, -RC_MAX_ROLL_RATE, +RC_MAX_ROLL_RATE); //To-Do: Make this min/max controllable by the remote
    req_roll = map(unRollIn, MINRC_ROLL, MAXRC_ROLL, +RC_MAX_ROLL_RATE, -RC_MAX_ROLL_RATE); //To-Do: Make this min/max controllable by the remote  

    //Update the PID INPUTS (ie what is actually happening)
    pidInput[PID_PITCH_RATE] = 0.6 * gyro[0] + 0.4 * m1_gyro[0]; //Takes a weighted average of the gryo readings
    pidInput[PID_ROLL_RATE] = 0.6 * gyro[1] + 0.4 * m1_gyro[1]; //Takes a weighted average of the gryo readings
    pidInput[PID_YAW_RATE] = 0.6 * gyro[2] + 0.4 * m1_gyro[2]; //Takes a weighted average of the gryo readings

    pidInput[PID_PITCH_STAB] = yrp[2];
    pidInput[PID_ROLL_STAB] = yrp[1];
    pidInput[PID_YAW_STAB] = yrp[0] - yaw_target;  

    //Correct error to take the "shortest" route
    if (pidInput[PID_YAW_STAB] < -180) {
      pidInput[PID_YAW_STAB]+=360;
    }
    else if (pidInput[PID_YAW_STAB] > 180) {
      pidInput[PID_YAW_STAB]-=360;
    }

    // ================================================================
    // ===                  Determine Quad Mode (Acro/Stab)         ===
    // ================================================================

    //Update the PID SetPoints (targets)
    if (quadMode == ACRO_MODE)
    {
      pid_PITCH_STAB.SetMode(MANUAL);
      pid_ROLL_STAB.SetMode(MANUAL);

      pidSetPoint[PID_PITCH_RATE] = req_pitch;
      pidSetPoint[PID_ROLL_RATE] = req_roll;
    }
    else if (quadMode == STAB_MODE)
    {
      pid_PITCH_STAB.SetMode(AUTOMATIC);
      pid_ROLL_STAB.SetMode(AUTOMATIC);

      //Convert controller input into a desired angle
      req_pitch = map(unPitchIn, MINRC_PITCH, MAXRC_PITCH, -450, +450) / 10.0; //To-Do: Make this min/max controllable by the remote?
      req_roll = map(unRollIn, MINRC_ROLL, MAXRC_ROLL, -450, +450) / 10.0; //To-Do: Make this min/max controllable by the remote?

      //Tell the Stab pids what we want
      pidSetPoint[PID_PITCH_STAB] = req_pitch;
      pidSetPoint[PID_ROLL_STAB] = req_roll;

      //Compute Stabiliser PIDs...
      pid_PITCH_STAB.Compute();
      pid_ROLL_STAB.Compute();

      //...then set the pidSetPoint of the RATE PIDs to be the output of the stabiliser PIDs
      pidSetPoint[PID_PITCH_RATE] = pidOutput[PID_PITCH_STAB];
      pidSetPoint[PID_ROLL_RATE] = -pidOutput[PID_ROLL_STAB];
    }

    // ================================================================
    // ===                  Calculate YAW required                  ===
    // ================================================================

    pidSetPoint[PID_YAW_STAB] = 0.0;  
    pid_YAW_STAB.Compute();

    //If pilot asking for a yaw change - feed directly to rate PID (overwriting yaw stab output)
    if (abs(req_yaw) > 15)
    {
      pidOutput[PID_YAW_STAB] = req_yaw;
      yaw_target = yrp[0];  //Remeber yaw for when pilot stops input
    }

    //Set YAW rate PID setpoint
    pidSetPoint[PID_YAW_RATE] = pidOutput[PID_YAW_STAB];

    // ================================================================
    // ===             Compute Rate PIDS and send to Motors         ===
    // ================================================================

    //Calculate PIDs
    pid_PITCH_RATE.Compute();
    pid_ROLL_RATE.Compute();
    pid_YAW_RATE.Compute();

    //Un-comment the below lines if on a rig that only allows certain movement (when calibrating PIDS)
    //pidOutput[PID_YAW_RATE] = 0.0;
    //pidOutput[PID_PITCH_RATE] = 0.0;
    //pidOutput[PID_ROLL_RATE] = 0.0;

    //Get Outputs and write to motors
    esc_FL = constrain(rc_throttle - pidOutput[PID_ROLL_RATE] - pidOutput[PID_PITCH_RATE] - pidOutput[PID_YAW_RATE], MIN_ESC, MAX_ESC);
    esc_BL = constrain(rc_throttle - pidOutput[PID_ROLL_RATE] + pidOutput[PID_PITCH_RATE] + pidOutput[PID_YAW_RATE], MIN_ESC, MAX_ESC);
    esc_FR = constrain(rc_throttle + pidOutput[PID_ROLL_RATE] - pidOutput[PID_PITCH_RATE] + pidOutput[PID_YAW_RATE], MIN_ESC, MAX_ESC);
    esc_BR = constrain(rc_throttle + pidOutput[PID_ROLL_RATE] + pidOutput[PID_PITCH_RATE] - pidOutput[PID_YAW_RATE], MIN_ESC, MAX_ESC);  

    motorFL.writeMicroseconds(esc_FL);
    motorBL.writeMicroseconds(esc_BL);
    motorFR.writeMicroseconds(esc_FR);
    motorBR.writeMicroseconds(esc_BR);

    // ================================================================
    // ===             Debug section - Write to serial port         ===
    // ================================================================

    if (millis() - lastSerialDebug > serialDebug) {
      lastSerialDebug = millis();

      //      Serial.print("Yaw: \t");
      //      Serial.print(yrp[0]);
      //
      //      Serial.print("\tYaw Target: ");
      //      Serial.print(yaw_target);
      //
      //      Serial.print("\tYaw error: ");
      //      Serial.print(yrp[0] - yaw_target);
      //
      //      Serial.print("\tCorrected Error: ");
      //      Serial.print(pidInput[PID_YAW_STAB]);
      //
      Serial.print("Yaw Rate p: \t");      
      Serial.print(pid_YAW_RATE.GetKp(), 4);
      Serial.print("\tYaw Rate i: \t");      
      Serial.print(pid_YAW_RATE.GetKi(), 4);
      Serial.print("\tYaw Stab p: \t");      
      Serial.println(pid_YAW_STAB.GetKp(), 4);        
      //
      //      Serial.print("P:\t");
      //      Serial.print(rollAndPitchRate_P,4);
      //      Serial.print("\tI:\t");
      //      Serial.print(rollAndPitchRate_I,4);
      //      Serial.print("\tD:\t");
      //      Serial.print(rollAndPitchRate_D,7);   
      //      Serial.print("\tStab P:\t");
      //      Serial.println(rollAndPitchStab_P,4);
      //
      //      Serial.print("Free memory:\t");
      //      Serial.println(freeMemory());
    }
  }
  else {
    //Turn motors off ('slowly', otherwise they seem to stay on for 1 second)

    if (esc_FL > 700) { 
      esc_FL-=5;
    }
    if (esc_FR > 700) { 
      esc_FR-=5;
    }
    if (esc_BL > 700) { 
      esc_BL-=5;
    }
    if (esc_BR > 700) { 
      esc_BR-=5;
    }

    motorFL.writeMicroseconds(esc_FL);
    motorFR.writeMicroseconds(esc_FR); 
    motorBL.writeMicroseconds(esc_BL); 
    motorBR.writeMicroseconds(esc_BR);

    //Turn the PIDS off
    pid_PITCH_RATE.SetMode(MANUAL);
    pid_PITCH_STAB.SetMode(MANUAL);
    pid_ROLL_RATE.SetMode(MANUAL);
    pid_ROLL_STAB.SetMode(MANUAL);
    pid_YAW_STAB.SetMode(MANUAL);
    pid_YAW_RATE.SetMode(MANUAL);

    //Store this YAW for when we take off again
    yaw_target = yrp[0];

    // ================================================================
    // ===             Method to store what "FLAT" is to EEPROM     ===
    // ================================================================

    //Place the Arduino on a perfectly flat surface.
    //Before the Green light comes on to let you know that the YAW is stable, push Pitch forwards and move Roll right
    //Maintain stick position for 2 second

    //Start request detection
    req_pitch = map(unPitchIn, MINRC_PITCH, MAXRC_PITCH, -1000, +1000); //To-Do: Make this min/max controllable by the remote
    req_roll = map(unRollIn, MINRC_ROLL, MAXRC_ROLL, -1000, +1000); //To-Do: Make this min/max controllable by the remote  

    if (yaw_stable == false && requestStart != true && req_pitch > 500 && req_roll > 500)
    {
      requestStart = true;
      requestStartMillis = millis(); 
    }
    else if (yaw_stable == false && requestStart == true && valuesSaved == false && saveWhenStable == false && req_pitch > 500 && req_roll > 500 && millis() - requestStartMillis > 2000)
    {
      //If request for more than 1 second - save the current roll/pitch angles in the EEPROM
      saveWhenStable = true;
      Serial.println(F("Request made to save offsets when stable. Do not move Quad")); 
    }

  }

  // ================================================================
  // ===           Graphing section - write to serial port        ===
  // ================================================================

  //  Serial.print(gyro[1]);
  //  Serial.print("\t");
  //  Serial.print(yrp[1],7);
  //  Serial.print("\t");
  //  Serial.print(0);
  //  Serial.print("\t");
  //  Serial.print(0);
  //  Serial.print("\t");
  //  Serial.print(benchmarkResult);
  //  Serial.println("\t");
}

//Function to save pitch/roll offsets in EEPROM
void saveOffsets()
{
  byte bufferUpdated = GetLatestPacket();
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(yrp, &q, &gravity);

  rollOffset = yrp[1] * -180/M_PI;
  pitchOffset = yrp[2] * -180/M_PI;

  EEPROM.updateFloat(0, rollOffset); //Store current roll (minus this off actual roll to get "LEVEL")
  EEPROM.updateFloat(4, pitchOffset); //Store current roll (minus this off actual roll to get "LEVEL")

  valuesSaved = true;      

  Serial.println(F("Values stored in EEPROM")); 
}

// simple interrupt service routine
void calcThrottle()
{
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if(digitalRead(THROTTLE_IN_PIN) == HIGH)
  { 
    ulThrottleStart = micros();
  }
  else
  {
    // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
    // this gives use the time between the rising and falling edges i.e. the pulse duration.
    unThrottleInShared = (uint16_t)(micros() - ulThrottleStart);
    // use set the throttle flag to indicate that a new throttle signal has been received
    bUpdateFlagsShared |= THROTTLE_FLAG;
  }
}

void calcPitch()
{
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if(digitalRead(PITCH_IN_PIN) == HIGH)
  { 
    ulPitchStart = micros();
  }
  else
  {
    // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
    // this gives use the time between the rising and falling edges i.e. the pulse duration.
    unPitchInShared = (uint16_t)(micros() - ulPitchStart);
    // use set the throttle flag to indicate that a new throttle signal has been received
    bUpdateFlagsShared |= PITCH_FLAG;
  }
}

void calcYaw()
{
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if(digitalRead(YAW_IN_PIN) == HIGH)
  { 
    ulYawStart = micros();
  }
  else
  {
    // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
    // this gives use the time between the rising and falling edges i.e. the pulse duration.
    unYawInShared = (uint16_t)(micros() - ulYawStart);
    // use set the throttle flag to indicate that a new throttle signal has been received
    bUpdateFlagsShared |= YAW_FLAG;
  }
}

void calcRoll()
{
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if(digitalRead(ROLL_IN_PIN) == HIGH)
  { 
    ulRollStart = micros();
  }
  else
  {
    // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
    // this gives use the time between the rising and falling edges i.e. the pulse duration.
    unRollInShared = (uint16_t)(micros() - ulRollStart);
    // use set the throttle flag to indicate that a new throttle signal has been received
    bUpdateFlagsShared |= ROLL_FLAG;
  }
}

void calcHT()
{
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if(digitalRead(HT_IN_PIN) == HIGH)
  { 
    ulHTStart = micros();
  }
  else
  {
    // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
    // this gives use the time between the rising and falling edges i.e. the pulse duration.
    unHTInShared = (uint16_t)(micros() - ulHTStart);
    // use set the throttle flag to indicate that a new throttle signal has been received
    bUpdateFlagsShared |= HT_FLAG;
  }
}

void calcHP()
{
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if(digitalRead(HP_IN_PIN) == HIGH)
  { 
    ulHPStart = micros();
  }
  else
  {
    // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
    // this gives use the time between the rising and falling edges i.e. the pulse duration.
    unHPInShared = (uint16_t)(micros() - ulHPStart);
    // use set the throttle flag to indicate that a new throttle signal has been received
    bUpdateFlagsShared |= HP_FLAG;
  }
}

void calcPT()
{
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if(digitalRead(PT_IN_PIN) == HIGH)
  { 
    ulPTStart = micros();
  }
  else
  {
    // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
    // this gives use the time between the rising and falling edges i.e. the pulse duration.
    unPTInShared = (uint16_t)(micros() - ulPTStart);
    // use set the throttle flag to indicate that a new throttle signal has been received
    bUpdateFlagsShared |= PT_FLAG;
  }
}

void calcAux()
{
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if(digitalRead(AUX_IN_PIN) == HIGH)
  { 
    ulAuxStart = micros();
  }
  else
  {
    // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
    // this gives use the time between the rising and falling edges i.e. the pulse duration.
    unAuxInShared = (uint16_t)(micros() - ulAuxStart);
    // use set the throttle flag to indicate that a new throttle signal has been received
    bUpdateFlagsShared |= AUX_FLAG;
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



















