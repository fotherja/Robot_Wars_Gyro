/*
To Do:
1) Perhaps include some method to allow for PID tuning on the fly...?
2) Perhaps detect if robot is not on the floor and temporaily disable PID control if so


A) Rename routines to something proper
B) Rename FwdBck etc in the interrupt routines to make more sense!

We're going for a rate of turn approach. The steering stick will set the rate of turn and the robot will attempt to follow.
stick local variables everywhere instead of globals!

There are a few options on how to control the bot:
 - right stick direction of heading and speed
 - Rate of turn and speed
 - A dial that sets the setpoint

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
const int BATTERY_THRESHOLD_LOW = 6400;                                           // If voltage falls below this, enter sleep
const int BATTERY_THRESHOLD_HGH = 6800;                                           // If voltage rises above this, turn active again
const int NO_SIGNAL_THESHOLD = 10;                                                // Number of erronous PPM signals received before we deduce no signal is being received

const int VALID = 1;
const int INVALID = 0;

static int No_Signal = NO_SIGNAL_THESHOLD;                                        // Assume no signal at startup
 
//--- Pin definitions: -----------------------------------------------------------
const int LED = 0;                                                                // LED Pin. Although this is also the Rx pin

const int FwdBckIn = 1;                                                           // This is PCINT17 (calls PCINT2_vect). Although this is also the Tx pin
const int LfRghtIn = 3;                                                           // This is INT1

const int MEnble = 8;                                                             // Enable Motors
const int MotorL = 9;                                                             // Left Motors
const int MotorR = 10;                                                            // Right Motors

const int Vsense = A2;                                                            // Voltage sensing (150ohm to 1K divider)

//--- Globals: -------------------------------------------------------------------
volatile signed long FwdBckPulseWdth;                                             // The interrupt routines continously update these whenever a new PPM signal is received (hence volatile!)
volatile signed long LfRghtPulseWdth;

signed long FwdBckPulseWdth_Safe;                                                 // The Process_PPM routine saves the volatile versions above to these safe copies for later modification and use  
signed long LfRghtPulseWdth_Safe;

float Yaw, Yaw_setpoint;                                                          // The measured Yaw and the desired Yaw provided by the user's stick movements (respectively)

//--- Routines -------------------------------------------------------------------
int Low_Battery(void);                                                            // Returns 0 if battery is sufficiently charged, 1 otherwise. Includes hysterisis & LED flashing for low battery (Non-blocking)
void setup_6050(void);                                                            // Initialises the DMP6050
void PID_Routine(void);                                                           // Iterates PID algorithm if enough time has passed since last call and updates the PWM signals to the motors
float Turn_Error(void);                                                           // From the Yaw & Yaw_setpoint, this routine returns how many degrees we are from where we want to be. 
int Process_PPM(void);                                                            // Validates and processes PPM signals received by the interrupt routines. Updates Yaw_setpoint. Returns 0 if no PPM signal. 

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

  pinMode(LED, OUTPUT); 
}

//--- Main: ----------------------------------------------------------------------
void loop() {
    // If DMP initialisation failed just run without gyro control
    if (!dmpReady)  {
      while(1)  {
        if(!Low_Battery())  {                                                     // If the battery isn't flat...
          if(Process_PPM() == VALID) {                                            // If we're recieving valid PPM data... 
            
            digitalWrite(MEnble, HIGH);                                           // Ensure outputs are enabled and go on to update them
            int Lout = constrain(128 + FwdBckPulseWdth_Safe + LfRghtPulseWdth_Safe, 0, 255);         
            int Rout = constrain(128 - FwdBckPulseWdth_Safe + LfRghtPulseWdth_Safe, 0, 255);         
            OCR1A = Lout;
            OCR1B = Rout;
          }          
        } 
      }
    }

    while (!mpuInterrupt && fifoCount < packetSize) {                             // wait for MPU interrupt or extra packet(s) available          
      if(!Low_Battery())  {        
        if(Process_PPM() == VALID) {                                              // Process_PPM() returns 1 if valid PPM data is being received 
          PID_Routine();
        }         
      }         
    }
    
    mpuInterrupt = false;                                                         // reset interrupt flag and get INT_STATUS byte
    mpuIntStatus = mpu.getIntStatus();
    
    fifoCount = mpu.getFIFOCount();                                               // get current FIFO count
    
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {                             // check for overflow (this should never happen unless our code is too inefficient)      
      mpu.resetFIFO();                                                            // reset so we can continue cleanly      
    }    
    else if (mpuIntStatus & 0x02) {                                               // otherwise, check for DMP data ready interrupt (this should happen frequently)        
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();              // wait for correct available data length, should be a VERY short wait
        mpu.getFIFOBytes(fifoBuffer, packetSize);                                 // read a packet from FIFO
        
        fifoCount -= packetSize;                                                  // track FIFO count here in case there is > 1 packet available (this lets us immediately read more without waiting for an interrupt)
        
        mpu.dmpGetQuaternion(&q, fifoBuffer);                                     // display Euler angles in degrees
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);        
        Yaw = (ypr[0] * 180.0/M_PI) + 180.0;                                      // Yaw given as a value between 0-360             
    }
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//#######################################################################################################################################################################
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
int Process_PPM() 
{  
// A few things about this routine:
//  1) It's run every 40ms
//  2) It validates PWM data on Ch1 & Ch2 and sets PWM outputs to motors accordingly 
//  3) Turns LED on to indiciate circuit is receiving valid PWM from receiver     

  // 1) -----  
  static int LfRghtPulse;
  static int FwdBckPulse;
  static unsigned long Time_at_Motor_Update;
  
  if(millis() - Time_at_Motor_Update < 40) { 
    if(No_Signal > NO_SIGNAL_THESHOLD)  {
      return(0);
    }
    else  {
      return(1);  
    }    
  }  
  Time_at_Motor_Update = millis(); 


  // 2) -----
  if((FwdBckPulseWdth >= 900) && (FwdBckPulseWdth <= 2100)) {                     // If we have a pulse within valid range
      FwdBckPulseWdth_Safe = FwdBckPulseWdth - 1500;
      FwdBckPulseWdth = 0;                                                        // If no new values are received this ensures we don't keep using old PWM data
      
      FwdBckPulseWdth_Safe = FwdBckPulseWdth_Safe / 2;
      FwdBckPulse = VALID;                  
    }
  else  {  
      FwdBckPulse = INVALID;
      No_Signal++;        
    }
  
  if((LfRghtPulseWdth >= 900) && (LfRghtPulseWdth <= 2100)) {                     // If we have a pulse within valid range
      LfRghtPulseWdth_Safe = LfRghtPulseWdth - 1500;
      LfRghtPulseWdth = 0;
      
      Yaw_setpoint += (((float)LfRghtPulseWdth_Safe) * 0.05);                     // Scaling so full stick (+500us) gives 500 degrees per second rate of turn
      LfRghtPulseWdth_Safe = LfRghtPulseWdth_Safe / 2;     
      LfRghtPulse = VALID;   
    }
  else  {
      LfRghtPulse = INVALID;
      No_Signal++;
    }                                                             
       

  if(Yaw_setpoint >= 360.0) {                                                     // Keep Yaw_setpoint within 0-360 limits
      Yaw_setpoint -= 360.0;
    }
  else if(Yaw_setpoint < 0.0) {
      Yaw_setpoint += 360.0;
    }      

  if(FwdBckPulse && LfRghtPulse)  {                                               // If we've just received 2 valid pulses, clear the No_Signal count    
      No_Signal = 0;  
    }      
   

  // 3) -----
  if(No_Signal > NO_SIGNAL_THESHOLD)  {     
    digitalWrite(LED, LOW);                                                       // Turn LED off if no signal is being received
    digitalWrite(MEnble, LOW);                                                    // Disable outputs  
           
    OCR1A = 128;
    OCR1B = 128; 
    return(0);
  } 
  else  {
    digitalWrite(LED, HIGH);                                                      // Turn LED on to indicate signal being received   
    return(1);
  }
}

//--------------------------------------------------------------------------------
void PID_Routine()
{
// So we need LfRghtPulseWdth to adjust itself according to how much error there is. Here's a little PI controller to try out.
// Things to adjust:
// Kp, Ki & Kd
// The constraints
// The sample rate

  static unsigned long Time_at_Motor_Update;
  unsigned long delta_t;

  delta_t = micros() - Time_at_Motor_Update;
  
  if(delta_t < 5000) { 
    return;
  }  
  Time_at_Motor_Update = micros(); 


  float Error, Output, dInput;
  static float ErrorSum, LastYaw; 
  const float Kp = 1.25;                                                          // And error of 30 degrees contributes 37.5 to the output
  const float Ki = 8.0e-7;                                                        // And error of 30 degrees contributes to 24 to the output per second
  const float Kd = 1.0e5;                                                         // 90 degree/second contributes -9 to the output    
  
  Error = Turn_Error();
  
  ErrorSum += (Error * Ki) * delta_t;                                             // delta_t ~= 5000                                                       
  ErrorSum = constrain(ErrorSum, -100.0, 100.0);

  dInput = (Yaw - LastYaw) / delta_t;
  LastYaw = Yaw;
  
  Output = (Error * Kp) + ErrorSum - (dInput * Kd);
  
  LfRghtPulseWdth_Safe = round(Output);
  LfRghtPulseWdth_Safe = constrain(LfRghtPulseWdth_Safe, -100, 100);

  digitalWrite(MEnble, HIGH);                                                     // Ensure outputs are enabled
  int Lout = constrain(128 + FwdBckPulseWdth_Safe + LfRghtPulseWdth_Safe, 0, 255);         
  int Rout = constrain(128 - FwdBckPulseWdth_Safe + LfRghtPulseWdth_Safe, 0, 255);         
  OCR1A = Lout;
  OCR1B = Rout;  
}

//--------------------------------------------------------------------------------
float Turn_Error()
{
// This routine calculates the angular error to the setpoint. It returns (+ve) for clockwise or (-ve) for anticlockwise directions

  float difference = Yaw_setpoint - Yaw;
  float Error; 
  
  difference = abs(difference);

  if(Yaw_setpoint >= 180.0 && Yaw >= 180.0)
    {
      if(Yaw_setpoint > Yaw) {
        Error = Yaw_setpoint - Yaw;
        return Error;
      }
      else {
        Error = Yaw - Yaw_setpoint;
        return -Error;
      } 
    }
    
  else if(Yaw_setpoint >= 180.0 && Yaw < 180.0)
  {
      if(difference < 180.0) {
        Error = Yaw_setpoint - Yaw;
        return Error;
      }
      else {
        Error = 360.0 - Yaw_setpoint + Yaw;
        return -Error;
      }    
  } 
    
  else if(Yaw_setpoint < 180.0 && Yaw < 180.0)
  {
      if(Yaw_setpoint > Yaw) {
        Error = Yaw_setpoint - Yaw;
        return Error;
      }
      else {
        Error = Yaw - Yaw_setpoint;
        return -Error; 
      }   
  }  

  else if(Yaw_setpoint < 180.0 && Yaw >= 180.0)
  {
      if(difference < 180.0) {
        Error = Yaw - Yaw_setpoint;
        return -Error;
      }
      else {
        Error = 360 - Yaw + Yaw_setpoint;
        return Error;
      }    
  }  
}

//--------------------------------------------------------------------------------
int Low_Battery()
{
// Returns 0 if battery is sufficiently charged, 1 otherwise. Looks complicated because it also has hysterisis and fast flashes the LED to indicate the problem
  
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
    digitalWrite(LED, !digitalRead(LED));                                         // Flash LED
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




