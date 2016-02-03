/*
To Do:
Need to allow low voltage to work with gyro. ATM it would cause problems 

Keep track of angle rotated from startup reference. Will probs have to use 500 maybe 1000dps mode on IMU.
There are many options on how to control the bot:
 - right stick direction of heading and speed
 - right stick direction, left stick speed.
 - left l/r stick rate of turn. right stick speed
 - Somehow connect up a dial and use this to turn the bot.

*/
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

//--- IMU6050 Specific: ----------------------------------------------------------
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

//--- Defines: -------------------------------------------------------------------
const float VOLTAGE_SENSE_CONSTANT = 24.71;
const int BATTERY_THRESHOLD_LOW = 6600;                                           // If voltage falls below this, enter sleep
const int BATTERY_THRESHOLD_HGH = 6800;                                           // If voltage rises above this, turn active again

const int VALID = 1;
const int INVALID = 0;
 
//--- Pin definitions: -----------------------------------------------------------
const int LED = 0;                                                                // LED Pin. Although this is also the Rx pin

const int FwdBckIn = 1;                                                           // This is PCINT17 (calls PCINT2_vect). Although this is also the Tx pin
const int LfRghtIn = 3;                                                           // This is INT1

const int MEnble = 8;                                                             // Left Motors
const int MotorL = 9;                                                             // Right Motors

const int MotorR = 10;                                                            // Enable Motors
const int Vsense = A2;                                                            // Voltage sensing (150ohm to 1K dividor

//--- Globals: -------------------------------------------------------------------
volatile signed long FwdBckPulseWdth;
volatile signed long LfRghtPulseWdth;

volatile unsigned long Time_at_Rise_LR;
volatile unsigned long Time_at_Rise_FB;
volatile unsigned long Time_at_Motor_Update;

int Lout;
int Rout;

int No_Signal;
int debug_count;

int LfRghtPulse, FwdBckPulse;

float yaw;
//--- Routines -------------------------------------------------------------------
void Low_Battery(void);
void setup_6050(void);
void Error_loop(void);

//--- Setup: ---------------------------------------------------------------------
void setup() 
{  
  setup_6050();
  
  //Setup timer1 Registers
  TCCR1A = 0b11110001;                                                            // Fast PWM 125KHz (8 bit)
  TCCR1B = 0b00001001;
  OCR1A = 128;
  OCR1B = 128;  
  
  //Initialise Pin directions:
  pinMode(MotorL, OUTPUT);  
  pinMode(MotorR, OUTPUT); 
  
  pinMode(MEnble, OUTPUT);

  pinMode(FwdBckIn, INPUT_PULLUP);
  pinMode(LfRghtIn, INPUT_PULLUP);  

  attachInterrupt (digitalPinToInterrupt(3), ISR_LR, CHANGE);                     // Attach interrupt handler
  PCMSK2 = 0b00000010;                                                            // Allow interrupt only PCINT17
  PCICR = 0b00000100;                                                             // Enable Interrupt on PCI_2
    
  Serial.begin(38400); 

  pinMode(LED, OUTPUT); 
}

//--- Main: ----------------------------------------------------------------------
void loop() {
    // If DMP initialisation failed just run without gyro control
    if (!dmpReady) 
      {
        while(1)  {
          Update_Motors();          
        }
      }

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {           
          Update_Motors();        
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();        

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);        
        yaw = (ypr[0] * 180/M_PI);             
    }
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//#######################################################################################################################################################################
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Update_Motors() 
{  
// A few things about this routine:
//  1) It's run every ~40ms
//  2) It Monitors Battery voltage and shutsdown motors if it falls below a threshold voltage
//  3) It validates PWM data on Ch1 & Ch2 and set PWM outputs to motors accordingly - 
//  4) Turns LED on to indiciate circuit is receiving valid PWM from receiver     

  // 1) -----
  if(millis() - Time_at_Motor_Update < 40)
  {
    return;
  }
  
  Time_at_Motor_Update = millis();
  
  // 2) -----
  float BVoltage = analogRead(Vsense) * VOLTAGE_SENSE_CONSTANT; //Convert to millivolts
  if(BVoltage < BATTERY_THRESHOLD_LOW)
    {
      Low_Battery();
    }

  // 3) -----
  if((FwdBckPulseWdth >= 900) && (FwdBckPulseWdth <= 2100))                       // If we have a pulse within valid range
    {
      FwdBckPulseWdth = FwdBckPulseWdth - 1500;
      FwdBckPulseWdth = FwdBckPulseWdth / 2;
      FwdBckPulse = VALID;                  
    }
  else
    {
      FwdBckPulse = INVALID;
      No_Signal++;        
    }
  
  if((LfRghtPulseWdth >= 900) && (LfRghtPulseWdth <= 2100))                       // If we have a pulse within valid range
    {
      LfRghtPulseWdth = LfRghtPulseWdth - 1500;
      LfRghtPulseWdth = LfRghtPulseWdth / 4;     
      LfRghtPulse = VALID;   
    }
  else
    {
      LfRghtPulse = INVALID;
      No_Signal++;
    }

  if(FwdBckPulse && LfRghtPulse)                                                  // If we've just received 2 valid pulses, update motors
    {
      digitalWrite(MEnble, HIGH);                                                 // Ensure outputs are enabled
      Lout = constrain(128 + FwdBckPulseWdth + LfRghtPulseWdth, 0, 255);          // Left
      Rout = constrain(128 + FwdBckPulseWdth - LfRghtPulseWdth, 0, 255);          // Right 
      OCR1A = Lout;
      OCR1B = Rout;  
   
      No_Signal = 0;  
    }    
    
  FwdBckPulseWdth = 0;
  LfRghtPulseWdth = 0;    

  // 4) -----
  if(No_Signal > 10)
  {     
    digitalWrite(LED, LOW);                                                       // Turn LED off if no signal is being received
    digitalWrite(MEnble, LOW);                                                    // Disable outputs  
           
    OCR1A = 128;
    OCR1B = 128; 
  } 
  else
  {
    digitalWrite(LED, HIGH);                                                      // Turn LED on to indicate signal being received   
  }
}

//--------------------------------------------------------------------------------
void Error_loop()
{
  while(1)
  {
    //Using LED indicate the various errors here...
  }
}

//--------------------------------------------------------------------------------
void Low_Battery()
{
  digitalWrite(MEnble, LOW);                                                      //Disable all outputs
  while(1)
  {
    digitalWrite(LED, !digitalRead(LED));                                         // Flash LED
    delay(100);
    
    float BVoltage = analogRead(Vsense) * VOLTAGE_SENSE_CONSTANT;                 //Convert to millivolts
    if(BVoltage > BATTERY_THRESHOLD_HGH)
      {
        digitalWrite(MEnble, HIGH);                                               //Enable all outputs
        loop();
      }   
  }
}

//--------------------------------------------------------------------------------
void setup_6050() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif    

    mpu.initialize();

    // load and configure the DMP    
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready        
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection        
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it        
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } 
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//########## INTERRUPT DETECTION ROUTINES ###############################################################################################################################
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
ISR (PCINT2_vect)                                                                 // Change in FwdBckIn pin
{
  if (digitalRead(FwdBckIn) == HIGH)
    Time_at_Rise_FB = micros();
  else
    FwdBckPulseWdth = micros() - Time_at_Rise_FB; 
    
  PCIFR = 0b00000100;    // Apparently writing 1 clears the flag???
} 


void ISR_LR ()                                                                    // Change in LfRghtIn Pin
{
  if (digitalRead(LfRghtIn) == HIGH)
    Time_at_Rise_LR = micros();
  else
    LfRghtPulseWdth = micros() - Time_at_Rise_LR;        
}


void dmpDataReady() {
    mpuInterrupt = true;
}




