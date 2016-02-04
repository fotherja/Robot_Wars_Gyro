/*
To Do:
Ok. 2 Big problems at the moment in which I need a fresh mind to fix.
1) Any movement of the sticks seems to turn the LED off as if we're receiving invalid pulses...
2) Sometime the Robot tries to turn the wrong way to reach the setpoint...


We're going for a rate of turn approach. The steering stick will set the rate of turn and the robot will attempt to follow.
stick local variables everywhere instead of globals!


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

const int MEnble = 8;                                                             // Enable Motors
const int MotorL = 9;                                                             // Left Motors
const int MotorR = 10;                                                            // Right Motors

const int Vsense = A2;                                                            // Voltage sensing (150ohm to 1K dividor

//--- Globals: -------------------------------------------------------------------
volatile signed long FwdBckPulseWdth;
volatile signed long LfRghtPulseWdth;
signed long FwdBckPulseWdth_Safe;
signed long LfRghtPulseWdth_Safe;

float yaw, yaw_setpoint;

//--- Routines -------------------------------------------------------------------
int Low_Battery(void);
void setup_6050(void);
void PID_Routine(void);
float Turn_Error(void);

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
    
  //Serial.begin(38400); 

  pinMode(LED, OUTPUT); 
}

//--- Main: ----------------------------------------------------------------------
void loop() {
    // If DMP initialisation failed just run without gyro control
    if (!dmpReady) 
      {
        while(1)  {
          if(!Low_Battery())  {
            Update_Motors();         
          } 
        }
      }
    
    while (!mpuInterrupt && fifoCount < packetSize) {                             // wait for MPU interrupt or extra packet(s) available          
      if(!Low_Battery())  {
        Update_Motors();         
      }         
    }
    
    mpuInterrupt = false;                                                         // reset interrupt flag and get INT_STATUS byte
    mpuIntStatus = mpu.getIntStatus();
    
    fifoCount = mpu.getFIFOCount();                                               // get current FIFO count
    
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {                             // check for overflow (this should never happen unless our code is too inefficient)
        // reset so we can continue cleanly
        mpu.resetFIFO();        
    }    
    else if (mpuIntStatus & 0x02) {                                               // otherwise, check for DMP data ready interrupt (this should happen frequently)        
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();              // wait for correct available data length, should be a VERY short wait
        mpu.getFIFOBytes(fifoBuffer, packetSize);                                 // read a packet from FIFO
        
        fifoCount -= packetSize;                                                  // track FIFO count here in case there is > 1 packet available (this lets us immediately read more without waiting for an interrupt)
        
        mpu.dmpGetQuaternion(&q, fifoBuffer);                                     // display Euler angles in degrees
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);        
        yaw = (ypr[0] * 180/M_PI) + 180.0;                                        // Yaw given as a value between 0-360             
    }
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//#######################################################################################################################################################################
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Update_Motors() 
{  
// A few things about this routine:
//  1) It's run every 40ms
//  2) It validates PWM data on Ch1 & Ch2 and sets PWM outputs to motors accordingly 
//  3) Turns LED on to indiciate circuit is receiving valid PWM from receiver     

  // 1) -----
  static int No_Signal;
  static int LfRghtPulse;
  static int FwdBckPulse;
  static unsigned long Time_at_Motor_Update;
  
  if(millis() - Time_at_Motor_Update < 40) { 
    return;
  }
  
  Time_at_Motor_Update = millis(); 


  // 2) -----
  if((FwdBckPulseWdth >= 900) && (FwdBckPulseWdth <= 2100)) {                     // If we have a pulse within valid range
      FwdBckPulseWdth_Safe = FwdBckPulseWdth - 1500;
      FwdBckPulseWdth = 0;                                                        // If no new values are received this ensures we don't keep using old PWM data
      
      FwdBckPulseWdth_Safe = FwdBckPulseWdth_Safe / 4;
      FwdBckPulse = VALID;                  
    }
  else  {  
      FwdBckPulse = INVALID;
      No_Signal++;        
    }
  
  if((LfRghtPulseWdth >= 900) && (LfRghtPulseWdth <= 2100)) {                     // If we have a pulse within valid range
      LfRghtPulseWdth_Safe = LfRghtPulseWdth - 1500;
      LfRghtPulseWdth = 0;
      
      yaw_setpoint += (((float)LfRghtPulseWdth_Safe) * 0.05);                     // Scaling so full stick (+500us) gives 500 degrees per second rate of turn
      LfRghtPulseWdth_Safe = LfRghtPulseWdth_Safe / 4;     
      LfRghtPulse = VALID;   
    }
  else  {
      LfRghtPulse = INVALID;
      No_Signal++;
    }                                                             
       

  if(yaw_setpoint >= 360.0) {                                                     // Keep yaw_setpoint within 0-360 limits
      yaw_setpoint -= 360.0;
    }
  else if(yaw_setpoint < 0.0) {
      yaw_setpoint += 360.0;
    }      

  if(FwdBckPulse && LfRghtPulse)  {                                               // If we've just received 2 valid pulses, run the PID routine  
      PID_Routine();   
      No_Signal = 0;  
    }      
   

  // 3) -----
  if(No_Signal > 10)  {     
    digitalWrite(LED, LOW);                                                       // Turn LED off if no signal is being received
    digitalWrite(MEnble, LOW);                                                    // Disable outputs  
           
    OCR1A = 128;
    OCR1B = 128; 
  } 
  else  {
    digitalWrite(LED, HIGH);                                                      // Turn LED on to indicate signal being received   
  }
}

//--------------------------------------------------------------------------------
void PID_Routine()
{
// So we need LfRghtPulseWdth to adjust itself according to how much error there is. Lets try proportional control first.
  float Error;
  const float Kp = 2.0;    
 
  Error = Turn_Error() * Kp;
  //LfRghtPulseWdth_Safe = round(Error);  

  int Lout;
  int Rout;
  digitalWrite(MEnble, HIGH);                                                     // Ensure outputs are enabled
  Lout = constrain(128 + FwdBckPulseWdth_Safe + LfRghtPulseWdth_Safe, 75, 180);         
  Rout = constrain(128 - FwdBckPulseWdth_Safe + LfRghtPulseWdth_Safe, 75, 180);         
  OCR1A = Lout;
  OCR1B = Rout;  
}

//--------------------------------------------------------------------------------
float Turn_Error()
{
// This routine calculates the angular error to the setpoint. It returns (+ve) for clockwise or (-ve) for anticlockwise directions

  float difference = yaw_setpoint - yaw;
  float Error; 
  
  difference = abs(difference);

  if(yaw_setpoint >= 180.0 && yaw >= 180.0)
    {
      if(yaw_setpoint > yaw) {
        Error = yaw_setpoint - yaw;
        return Error;
      }
      else {
        Error = yaw - yaw_setpoint;
        return -Error;
      } 
    }
    
  else if(yaw_setpoint >= 180.0 && yaw < 180.0)
  {
      if(difference < 180.0) {
        Error = yaw_setpoint - yaw;
        return Error;
      }
      else {
        Error = 360.0 - yaw_setpoint + yaw;
        return -Error;
      }    
  } 
    
  else if(yaw_setpoint < 180.0 && yaw < 180.0)
  {
      if(yaw_setpoint > yaw) {
        Error = yaw_setpoint - yaw;
        return -Error;
      }
      else {
        Error = yaw - yaw_setpoint;
        return Error; 
      }   
  }  

  else if(yaw_setpoint < 180.0 && yaw >= 180.0)
  {
      if(difference < 180.0) {
        Error = yaw - yaw_setpoint;
        return -Error;
      }
      else {
        Error = 360 - yaw + yaw_setpoint;
        return Error;
      }    
  }  
}

//--------------------------------------------------------------------------------
int Low_Battery()
{
// Returns 0 if battery is sufficiently charged, 1 otherwise
  
  static unsigned long Time_at_LED_Change = 0;
  static bool Low_Battery_Flag = 0;
  
  float BVoltage = analogRead(Vsense) * VOLTAGE_SENSE_CONSTANT;                   //Convert to millivolts
  
  if(BVoltage > BATTERY_THRESHOLD_LOW && !Low_Battery_Flag)
    {
      return(0);
    }
  if(BVoltage > BATTERY_THRESHOLD_HGH)
    {
      Low_Battery_Flag = 0;
      return(0);
    }

  Low_Battery_Flag = 1;  
  digitalWrite(MEnble, LOW);                                                      //Disable all outputs

  if(millis() - Time_at_LED_Change > 100)
  {
    digitalWrite(LED, !digitalRead(LED));                                           // Flash LED
    Time_at_LED_Change = millis();
  }  
  
  return(1);  
}

//--------------------------------------------------------------------------------
void setup_6050() {
                                                                                  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24;                                                                // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif    

    mpu.initialize();                                                                                     
    devStatus = mpu.dmpInitialize();                                              // load and configure the DMP 
    
    mpu.setXGyroOffset(220);                                                      // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);                                                    // 1688 factory default for my test chip

    if (devStatus == 0) {                                                         // make sure it worked (returns 0 if so)                
        mpu.setDMPEnabled(true);                                                  // turn on the DMP, now that it's ready
              
        attachInterrupt(0, dmpDataReady, RISING);                                 // enable Arduino interrupt detection  
        mpuIntStatus = mpu.getIntStatus();
             
        dmpReady = true;                                                          // set our DMP Ready flag so the main loop() function knows it's okay to use it
       
        packetSize = mpu.dmpGetFIFOPacketSize();                                  // get expected DMP packet size for later comparison
    } 
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//########## INTERRUPT DETECTION ROUTINES ###############################################################################################################################
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
ISR (PCINT2_vect)                                                                 // Change in FwdBckIn pin
{
  static unsigned long Time_at_Rise_FB = 0;
  
  if (digitalRead(FwdBckIn) == HIGH)
    Time_at_Rise_FB = micros();
  else
    LfRghtPulseWdth = micros() - Time_at_Rise_FB;                                 // Yeah I know this is a bit confusing! Sorry. May change later? 
    
  PCIFR = 0b00000100;    // Apparently writing 1 clears the flag???
} 


void ISR_LR ()                                                                    // Change in LfRghtIn Pin
{
  static unsigned long Time_at_Rise_LR = 0;
  
  if (digitalRead(LfRghtIn) == HIGH)
    Time_at_Rise_LR = micros();
  else
    FwdBckPulseWdth = micros() - Time_at_Rise_LR;                                 
}


void dmpDataReady() {
    mpuInterrupt = true;
}





