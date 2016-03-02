/*
 * 
To Do:
1) Get variable IR bit length receiving working well!
2) Include some method to allow for PID tuning on the fly...? and EEPROM storage of these values (B, Y, X buttons) (R for decreasing them) 
  (Mode entered if start button held for 3 seconds) etc
4) Allow for single wire PPM input?
5) ESC support

Description:
  This program is the Code for the Gyro Motor Controller (GMC) board. 
  The board has a number of physical connections:
   - GND and Battery+ power inputs
   - 4 * motor outputs +/-. But there are only 2 motor channels
   - GND and 5v BEC to power a RF receiver if need be
   - 2 * PWM channel inputs from an RF receiver.
   - IR receiver (3.3v, GND, Signal)
  
  Obviously whether you need each set of connections depends on whether your using IR or RF.
   
  The GMC receives from a RF receiver if one is connected otherwise it resorts to receiving from an IR receiver. 
  
  All the GMC really does is act as a gyro stabilised mixer & motor driver for small robots. It uses a PID
  control algorithm running at 100Hz to to keep the robot pointing in the direction stored in the global variable, 
  Yaw_Setpoint. The remote control (IR or RF) changes this variable and the robot attempts to follow. The other input
  is the speed value, which just moves the robot back and forth.

  The PID algorithm has a number of tunable parameters that significantly effect the robots character.

  - PWM pulsewidths are measured using pinchange interrupts on each channel and the micros() function
  - IR data is read using pinchange interrupts and timer2. It expects manchester encoded data with a bit period of 0.8ms 
   (1 start bit, 8 Yaw_Setpoint bits followed by 8 Speed bits). It's pretty robust to spikes and errors.

  There is low battery detection (works for 1S or 2S LiPos) and LED indication of various states:
    - Low battery
    - No signal
    - Signal remaining in neutral so idling the motors (otherwise the PID control goes crazy when the robot is picked up...)

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
const float VOLTAGE_SENSE_CONSTANT = 10.08;                                       //24.71; - This is the value for the Green LED GMC that I built 1st
const int BATTERY_THRESHOLD_LOW_2S = 6300;                                        // If voltage falls below this, enter sleep
const int BATTERY_THRESHOLD_HGH_2S = 7000;                                        // If voltage rises above this, turn active again
const int BATTERY_THRESHOLD_LOW_1S = 3150;                                        // If voltage falls below this, enter sleep
const int BATTERY_THRESHOLD_HGH_1S = 3500;                                        // If voltage rises above this, turn active again

const int NO_SIGNAL_THESHOLD_PWM = 10;                                            // Number of erronous PPM signals received before we deduce no signal is being received
const int NO_SIGNAL_THESHOLD_IR = 10;                                             // Number of erronous PPM signals received before we deduce no signal is being received

const int VALID = 1;
const int INVALID = 0;
 
//--- Pin definitions: -----------------------------------------------------------
const int LED = 0;                                                                // LED Pin. Although this is also the Rx pin
const int ESC_PWM_Pin = 3;                                                        // Also output PWM

const int FwdBckIn = 3;                                                           // This is INT1
const int LfRghtIn = 1;                                                           // This is PCINT17 (calls PCINT2_vect). Although this is also the Tx pin

const int MEnble = 8;                                                             // Enable Motors
const int MotorL = 9;                                                             // Left Motors
const int MotorR = 10;                                                            // Right Motors

const int IR_Pos = 13;                                                            // Convienient use of pins to supply power to IR reciever.
const int IR_GND = 12;
const int IR_In  = 11;                                                            // PB3 which is PCINT3

const int Vsense = A2;                                                            // Voltage sensing (150ohm to 1K divider)

//--- Globals: -------------------------------------------------------------------
volatile long FwdBckPulseWdth;                                                    // The interrupt routines continously update these whenever a new PPM signal is received (hence volatile!)
volatile long LfRghtPulseWdth;

long FwdBckPulseWdth_Safe;                                                        // The Process_PPM routine saves the volatile versions above to these safe copies for later modification and use  
long LfRghtPulseWdth_Safe;

float Yaw, Yaw_setpoint;                                                          // The measured Yaw and the desired Yaw provided by the user's stick movements (respectively)

char RF_Receiver_Connected;

//--- IR Related:
volatile unsigned long Total_Time_High;                                           // Measures the time spent high during a bit_period. If Total_Time_High > 1/2 * Bit period we assume a 1 was sent. Filters spikes
volatile unsigned long TimeA, Time_Diff;
volatile unsigned long Encoded_Data[2];
volatile boolean Decode_Flag;

volatile int bit_index = 0;
volatile boolean State;

//--- Routines -------------------------------------------------------------------
int Low_Battery(void);                                                            // Returns 0 if battery is sufficiently charged, 1 otherwise. Includes hysterisis & LED flashing for low battery (Non-blocking)
void setup_6050(void);                                                            // Initialises the DMP6050
int PID_Routine(void);                                                            // Iterates PID algorithm if enough time has passed since last call and updates the PWM signals to the motors. Returns 1 if update performed
float Turn_Error(void);                                                           // From the Yaw & Yaw_setpoint, this routine returns how many degrees we are from where we want to be. 
int Process_PPM(void);                                                            // Validates and processes PPM signals received by the interrupt routines. Updates Yaw_setpoint. Returns 0 if no PPM signal. 
int Process_IR(void);                                                             // Validates and processes IR signals recieved by the interrupt routines. Updates Yaw_setpoint. Returns 0 if no IR signal.
long Decode(void);                                                                // Decodes the Manchester encoded received IR Data
void IR_OR_PWM(void);                                                             // Discovers whether we have an RF receiver or IR receiver connected at startup. The correct routines get called depending on this
void Beep_Motors(unsigned long Frequency, unsigned long Duration);                // Uses the motors as speakers to produce noise
void ESC_PWM(int Pulse_Width);                                                    // Call this regularly to keep a PWM signal being sent out the Rx pin 

//--- Setup: ---------------------------------------------------------------------
void setup() 
{  
  setup_6050();
  
  //Setup timer1 Registers for motor PWMing
  TCCR1A = 0b11110001;                                                            // Fast PWM 62.5KHz (8 bit)
  TCCR1B = 0b00001001;
  OCR1A = 128;
  OCR1B = 128;  
  
  //Initialise Pin directions:
  pinMode(MotorL, OUTPUT);  
  pinMode(MotorR, OUTPUT); 
  
  pinMode(MEnble, OUTPUT);

  pinMode(FwdBckIn, INPUT_PULLUP);
  pinMode(LfRghtIn, INPUT_PULLUP); 

  digitalWrite(IR_Pos, HIGH);  pinMode(IR_Pos, OUTPUT);                           // Power the IR Reciever using the I/Os themselves. The current is small so it's ok.
  digitalWrite(IR_GND, LOW);   pinMode(IR_GND, OUTPUT);
  pinMode(IR_In, INPUT_PULLUP);   

  //Configure Timer2 to interrupt every 0.4ms for IR receiving 
  TCCR2A = 0;  
  TCCR2B = 0;  
  TCNT2  = 0;  
  OCR2A  = 49;                                                                    // Interrupt every 0.4ms
  TCCR2A = 0b00000010;                                                            // Clear Timer on Compare match mode
  TCCR2B = 0b00000101;                                                            // ticks at 125KHz  
  TIMSK2 = 0b00000010;                                                            // Compare match on OCR2A interrupt

  attachInterrupt(digitalPinToInterrupt(3), ISR_LR, CHANGE);                      // Attach interrupt handler              (FwdBckIn)
  PCMSK0 = 0b00001000;                                                            // Allow interrupt only PCINT3           (IR_In)
  PCMSK2 = 0b00000010;                                                            // Allow interrupt only PCINT17          (LfRghtIn)
  PCICR = 0b00000101;                                                             // Enable Interrupt on PCI_2 and PCI_0

  pinMode(LED, OUTPUT);

  //Serial.begin(115200);
  
  IR_OR_PWM();                                                                    // Sets the RF_Receiver_Connected global variable to 1 if true otherwise 0 (BLOCKS until a signal is received)
  if(!RF_Receiver_Connected)
  {
    detachInterrupt(digitalPinToInterrupt(3));                                    // If No RF receiver attached disable the interrupts on these pins and use pin 3 for an ESC PWM signal
    PCMSK2 = 0b00000000; 
    pinMode(ESC_PWM_Pin, OUTPUT); 

    Beep_Motors(4000, 100);                                                       // Beep twice if IR being used (secondary control...)
    delay(100);
  }

  Beep_Motors(3000, 100);                                                         
}

//--- Main: ----------------------------------------------------------------------
void loop() {
    // If DMP initialisation failed just run without gyro control
    if (!dmpReady)  {
      while(1)  {
        if(!Low_Battery())  { 
          if(RF_Receiver_Connected) {                                             // If an RF receiver is connected take signals from this as it has better performance than an IR controller
            if(Process_PPM() == VALID)                                            // Process_PPM() returns 1 if valid PPM data is being received
            
              digitalWrite(MEnble, HIGH);                                         // Ensure outputs are enabled and go on to update them
              int Lout = constrain(128 + FwdBckPulseWdth_Safe + LfRghtPulseWdth_Safe, 0, 255);         
              int Rout = constrain(128 - FwdBckPulseWdth_Safe + LfRghtPulseWdth_Safe, 0, 255);         
              OCR1A = Lout;
              OCR1B = Rout;              
          }
          else  {                                                                 // Otherwise we assume an IR receiver is connected. If it's not we shutdown the motors anyway
            if(Process_IR() == VALID) {                                           // Process_IR() returns 1 if valid IR data is being received 
              
              digitalWrite(MEnble, HIGH);                                         // Ensure outputs are enabled and go on to update them
              int Lout = constrain(128 + FwdBckPulseWdth_Safe + LfRghtPulseWdth_Safe, 0, 255);         
              int Rout = constrain(128 - FwdBckPulseWdth_Safe + LfRghtPulseWdth_Safe, 0, 255);         
              OCR1A = Lout;
              OCR1B = Rout;              
            }                
          }
        }
      }
    }          

    while (!mpuInterrupt && fifoCount < packetSize) {                             // wait for MPU interrupt or extra packet(s) available          
      if(!Low_Battery())  { 
        if(RF_Receiver_Connected) {                                               // If an RF receiver is connected take signals from this as it has better performance than an IR controller
          if(Process_PPM() == VALID)                                              // Process_PPM() returns 1 if valid PPM data is being received 
            PID_Routine();
        }
        else  {                                                                   // Otherwise we assume an IR receiver is connected. If it's not we shutdown the motors anyway
          if(Process_IR() == VALID) {                                             // Process_IR() returns 1 if valid IR data is being received 
            PID_Routine();
          }                
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
//  1) It's run every 38ms
//  2) It validates PPM data on Ch1 & Ch2 
//  3) Turns LED on to indiciate circuit is receiving valid PPM from receiver     
//  4) If Sticks are in their neutral position for > 1 second, set Yaw_Setpoint = Yaw & stop motors

  // 1) -----  
  static int LfRghtPulse;
  static int FwdBckPulse;
  static int Neutral_Input_Count_PWM = 25;
  static unsigned long Time_at_Motor_Update; 
  static int No_Signal_PWM = NO_SIGNAL_THESHOLD_PWM;                              // Assume no signal at startup
  
  if(millis() - Time_at_Motor_Update < 38) { 
    if(No_Signal_PWM > NO_SIGNAL_THESHOLD_PWM || Neutral_Input_Count_PWM >= 25)  {
      digitalWrite(MEnble, LOW);
      Yaw_setpoint = Yaw;
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
      No_Signal_PWM++;        
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
      No_Signal_PWM++;
    }                                                             
       

  if(Yaw_setpoint >= 360.0) {                                                     // Keep Yaw_setpoint within 0-360 limits
      Yaw_setpoint -= 360.0;
    }
  else if(Yaw_setpoint < 0.0) {
      Yaw_setpoint += 360.0;
    }      

  if(FwdBckPulse && LfRghtPulse)  {                                               // If we've just received 2 valid pulses, clear the No_Signal count    
      No_Signal_PWM = 0;  
    }  

  // 3) -----
  if(No_Signal_PWM > NO_SIGNAL_THESHOLD_PWM)  {     
    digitalWrite(LED, LOW);                                                       // Turn LED off if no signal is being received
    digitalWrite(MEnble, LOW);                                                    // Disable outputs  
           
    OCR1A = 128;
    OCR1B = 128; 
    return(0);
  }   

  // 4) -----
  if(abs(LfRghtPulseWdth_Safe) <= 10 && abs(FwdBckPulseWdth_Safe) <= 10) {        // If control sticks are in their neurtal position...
    Neutral_Input_Count_PWM++;
    
    if(Neutral_Input_Count_PWM >= 25) {
      Yaw_setpoint = Yaw;
      digitalWrite(MEnble, LOW);
      
      if(Neutral_Input_Count_PWM % 15 == 0) {                                     // Slow flash LED to indicate the PID algorithm is suspended due to inactivity
        digitalWrite(LED, !digitalRead(LED));      
      }     
       
      return(0);                                                                  // Return with a zero to disable motor activity
    }   
  }
  else  {
    Neutral_Input_Count_PWM = 0; 
  }
  
  digitalWrite(LED, HIGH);                                                        // Turn LED on to indicate signal being received  
  return(1);  
}

//--------------------------------------------------------------------------------
int Process_IR() 
{ 
// A few things about this routine:
//  1) It's run every 38ms
//  2) It checks if any IR Data is being recieved. If it is it will always be returning a 1 otherwise a 0 
//  3) Turns LED on to indiciate circuit is receiving valid IR data from receiver  
//  4) If the joysticks go still for a while, idle.   

  // 1) ----- 
  static int Neutral_Input_Count_IR = 25;
  static unsigned long Time_at_Motor_Update; 
  static int No_Signal_IR = NO_SIGNAL_THESHOLD_IR;                                // Assume no signal at startup
  
  if(millis() - Time_at_Motor_Update < 38) { 
    if(No_Signal_IR > NO_SIGNAL_THESHOLD_IR || Neutral_Input_Count_IR >= 25)  {
      digitalWrite(MEnble, LOW);                                                  
      Yaw_setpoint = Yaw;
      return(0);
    }
    else  {
      return(1);  
    }    
  }  
  Time_at_Motor_Update = millis();


  FwdBckPulseWdth_Safe = 0; 
  LfRghtPulseWdth_Safe = 0; 
    
  if(Decode_Flag)  {                                                              // If IR Data has been received
    unsigned long IR_Data = Decode();                                             // Decode the data
    if(IR_Data)   {                                                               // If The data contains information clear the no signal count and extract the info
      No_Signal_IR = 0;
      
      Yaw_setpoint = (float)map(((IR_Data >> 8) & 0xFF), 0, 255, 0, 359);
      FwdBckPulseWdth_Safe = (IR_Data & 0xFF) - 128;
      FwdBckPulseWdth_Safe = (FwdBckPulseWdth_Safe * 3) / 4; 
       
      static float Old_Yaw_setpoint;
      LfRghtPulseWdth_Safe = round((Yaw_setpoint - Old_Yaw_setpoint) * 5.0);     // Incase the Gyro chip isn't populated this still allows for IR robot control
      Old_Yaw_setpoint = Yaw_setpoint;
    }
    else  
      No_Signal_IR++;                                                             // If the data packet was empty which occurs if there was an error etc, increase no signal count          
  }
  else  
    No_Signal_IR++;                                                               // Or if there was no data received at all, increase no signal count

  // 3) -----
  if(No_Signal_IR > NO_SIGNAL_THESHOLD_IR)  {     
    digitalWrite(LED, LOW);                                                       // Turn LED off if no signal is being received
    digitalWrite(MEnble, LOW);                                                    // Disable outputs  
           
    OCR1A = 128;
    OCR1B = 128; 
    return(0);
  }

  // 4) -----
  if(abs(LfRghtPulseWdth_Safe) <= 10 && abs(FwdBckPulseWdth_Safe) <= 10) {        // If control sticks are in their neurtal position...
    Neutral_Input_Count_IR++;
    
    if(Neutral_Input_Count_IR >= 25) {
      Yaw_setpoint = Yaw;
      digitalWrite(MEnble, LOW);
      
      if(Neutral_Input_Count_IR % 15 == 0) {                                      // Slow flash LED to indicate the PID algorithm is suspended due to inactivity
        digitalWrite(LED, !digitalRead(LED));      
      }     
             
      return(0);                                                                  // Return with a zero to disable motor activity
    }   
  }
  else  {
    Neutral_Input_Count_IR = 0; 
  }  
  
  digitalWrite(LED, HIGH);                                                        // Turn LED on to indicate signal being received ok  
  return(1);  
}

//--------------------------------------------------------------------------------
long Decode()
{
// Encoded_Data from ISR is processed to a long. If data is erronous return 0
                                                                            
  int i, index; 
  char BitH;
  char BitL;
  unsigned long IR_Data = 0;
  unsigned long Encoded_Data_Safe[2];
  
  Encoded_Data_Safe[0] = Encoded_Data[0];
  Encoded_Data_Safe[1] = Encoded_Data[1];
  Decode_Flag = 0;

  index = 15;
  for(i = 31; i >= 0;i -= 2)
  {
    BitH = bitRead(Encoded_Data_Safe[0], i);
    BitL = bitRead(Encoded_Data_Safe[0], i-1);

    if(BitH && !BitL)
      bitSet(IR_Data, index); 
    else if(!BitH && BitL)
      bitClear(IR_Data, index); 
    else
      return(0);                                                                  // Erronous data, ignore packet

    index--;
  }

  return(IR_Data);
}

//--------------------------------------------------------------------------------
int PID_Routine()
{
// So we need LfRghtPulseWdth to adjust itself according to how much error there is. Here's a little PID controller to do that. 
// It uses the global variable Yaw_Setpoint & need FwdBckPulseWdth_Safe for speed control
// Things to adjust:
// Kp, Ki & Kd
// The constraints
// The sample rate

  static unsigned long Time_at_Motor_Update;
  unsigned long delta_t;

  delta_t = micros() - Time_at_Motor_Update;  
  if(delta_t < 10000)                                                             // Run this routine every 10ms
    return(0); 
  Time_at_Motor_Update = micros();

  float Error, Output, dInput;
  static float ErrorSum, LastYaw; 
  const float Kp = 1.25;                                                          // And error of 30 degrees contributes ~37.5 to the output
  const float Ki = 4.0e-7;                                                        // And error of 30 degrees contributes to ~24 to the output per second
  const float Kd = 0.5e5;                                                         // 90 degree/second contributes -9 to the output    
  
  Error = Turn_Error();
  
  ErrorSum += (Error * Ki) * delta_t;                                             // delta_t ~= 10_000                                                     
  ErrorSum = constrain(ErrorSum, -100.0, 100.0);

  dInput = (Yaw - LastYaw) / delta_t;
  LastYaw = Yaw;
  
  Output = (Error * Kp) + ErrorSum - (dInput * Kd);
  
  LfRghtPulseWdth_Safe = round(Output);
  LfRghtPulseWdth_Safe = constrain(LfRghtPulseWdth_Safe, -100, 100);

  digitalWrite(MEnble, HIGH);                                                     // Ensure outputs are enabled
  unsigned char Lout = constrain(128 + FwdBckPulseWdth_Safe + LfRghtPulseWdth_Safe, 0, 255);         
  unsigned char Rout = constrain(128 - FwdBckPulseWdth_Safe + LfRghtPulseWdth_Safe, 0, 255);         
  OCR1A = Lout;
  OCR1B = Rout;

  return(1);
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
    
  if(Yaw_setpoint >= 180.0 && Yaw < 180.0)
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
    
  if(Yaw_setpoint < 180.0 && Yaw < 180.0)
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

  if(Yaw_setpoint < 180.0 && Yaw >= 180.0)
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
  static char Number_Of_Cells;
  static bool Low_Battery_Flag = 0;
  
  float BVoltage = analogRead(Vsense) * VOLTAGE_SENSE_CONSTANT;                   //Convert to millivolts

  // This section only runs the first time this routine is called. It detects how many cells the battery is made of
  static char Called_Before_Flag = 0;
  if(!Called_Before_Flag) {
    Called_Before_Flag = 1;

    if(BVoltage >= 5500)
      Number_Of_Cells = 2;
    else
      Number_Of_Cells = 1;
  }

  if(Number_Of_Cells == 2)  {
    if(BVoltage > BATTERY_THRESHOLD_LOW_2S && !Low_Battery_Flag)
      {
        return(0);
      }
    if(BVoltage > BATTERY_THRESHOLD_HGH_2S)
      {
        Low_Battery_Flag = 0;
        return(0);
      }
  }
  else  {
    if(BVoltage > BATTERY_THRESHOLD_LOW_1S && !Low_Battery_Flag)
      {
        return(0);
      }
    if(BVoltage > BATTERY_THRESHOLD_HGH_1S)
      {
        Low_Battery_Flag = 0;
        return(0);
      } 
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

//--------------------------------------------------------------------------------
void IR_OR_PWM()  
{
  delay(200);
  
  while(1)                                                                        // Keep looping until either an IR or RF signal is being received since there's no point continung otherwise!
  {    
    LfRghtPulseWdth = 0;
    FwdBckPulseWdth = 0;
    Decode_Flag = 0;
    
    delay(250);
  
    if(LfRghtPulseWdth && FwdBckPulseWdth)                                        // If we're receiving RF data return with RF_Receiver_Connected = 1
      RF_Receiver_Connected = 1;
      return;
      
    if(Decode_Flag) {
      if(Decode())  {                                                             // If we're receiving a proper manchester encoded signal from the IR return with RF_Receiver_Connected = 0;                         
        RF_Receiver_Connected = 0;
        return;      
      }
    }    
  } 
}

//--------------------------------------------------------------------------------
void Beep_Motors(unsigned long Frequency, unsigned long Duration)
{  
    digitalWrite(MEnble, HIGH);

    unsigned long Time_Period = 1000000 / Frequency;
    unsigned long Start_Time = millis();

    while(millis() - Start_Time < Duration)
    {
      OCR1A = 208;
      OCR1B = 208; 
      delayMicroseconds(Time_Period/2);      
      OCR1A = 48;
      OCR1B = 48; 
      delayMicroseconds(Time_Period/2);      
    }

    OCR1A = 128;
    OCR1B = 128;
    digitalWrite(MEnble, LOW);  
}

//--------------------------------------------------------------------------------
void ESC_PWM(int Pulse_Width)
{
  static unsigned long Time_Since_Last_Call;

  if(millis() - Time_Since_Last_Call < 20)
    return;
  Time_Since_Last_Call = millis();

  digitalWrite(ESC_PWM_Pin, HIGH);
  delayMicroseconds(Pulse_Width);
  digitalWrite(ESC_PWM_Pin, LOW);    
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//########## INTERRUPT DETECTION ROUTINES ###############################################################################################################################
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
ISR (PCINT2_vect)                                                                 // Change in LfRghtIn pin of PWM in
{
  static unsigned long Time_at_Rise_FB = 0;
  
  if (digitalRead(LfRghtIn) == HIGH)
    Time_at_Rise_FB = micros();
  else
    LfRghtPulseWdth = micros() - Time_at_Rise_FB;                                 
    
  PCIFR = 0b00000100;                                                             // Writing 1 clears the flag (Ridiciulous)
} 

//--------------------------------------------------------------------------------
void ISR_LR()                                                                     // Change in FwdBckIn Pin of PWM in
{
  static unsigned long Time_at_Rise_LR = 0;
  
  if (digitalRead(FwdBckIn) == HIGH)
    Time_at_Rise_LR = micros();
  else
    FwdBckPulseWdth = micros() - Time_at_Rise_LR;                                 
}

//--------------------------------------------------------------------------------
void dmpDataReady() {
    mpuInterrupt = true;
}

//--------------------------------------------------------------------------------
// This routine is for the IR data reciever. Once the start of a packet is detected, this routine works out if a 1 or 0 was sent after each bit period.
ISR (TIMER2_COMPA_vect)                                                           // IR dedicated timer compare match     
{  
  static unsigned long DataH, DataL;
  
  if(bit_index)
  {     
    bit_index--;
    
    Time_Diff = micros() - TimeA;
    TimeA = Time_Diff + TimeA;

    if(bit_index == 32)                                                           // To ignore the first start signal   
    {
      Total_Time_High = 0;
      return;
    }    
    
    if(State == HIGH) {
      Total_Time_High += Time_Diff;  
    }

    if(Total_Time_High > 200) {
      bitWrite(DataL, bit_index, 1);           
    }
    else  {
      bitWrite(DataL, bit_index, 0); 
    }   
    
    Total_Time_High = 0;

    if(!bit_index)  {
      Decode_Flag = 1;
      Encoded_Data[0] = DataL;               
    }
  }
} 

//--------------------------------------------------------------------------------
ISR (PCINT0_vect)                                                                 // IR_Pin Change
{   
  PCIFR = 0b00000001;                                                             // Writing 1 clears the flag (Ridiciulous)
  
  Time_Diff = micros() - TimeA;
  TimeA = Time_Diff + TimeA;

  State = digitalRead(IR_In);

  if(bit_index)                                                                   // If signal has started
  {
    if(State == LOW)                                                              //Signal must have just gone low so Time_Diff holds the most recent high pulse width
    {
      Total_Time_High += Time_Diff;
    }    
  } 

  // Here we detect whether the the signal has just started. If it has, set bit_index to EXPECTED_BITS*2 + 1
  if (!State && (Time_Diff > 20000))
  {   
    TCNT2 = 0;                                                                    // Reset the counter.      
    bit_index = 33; 
    TIFR2 = 0b00000010;                                                           //reset any counter interrupt too.        
    return;        
  }        
}



