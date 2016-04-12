/*
 * Title: Gyro Motor Controller
 * Author: James Fotherby
 * Date: 2/4/2016
 
Normally we send Type A Packets:
  DataA: Yaw_Setpoint (8 bits) - Speed (8 bits) - Packet Type A/B (1 bit)  
  DataB: PWM_Pulse_Width (8 bits) - Unused ATM (9 bits)
  
We only send DataB if there has been a change to the PWM_Pulse_Width, or button bits etc. So quite Rarely!
Things to know: Will the ESC work through a 4.7K resistor. Is it ok to loose signal completely or must it be at 1000us when not in use?

To Do:
1) Synchronise PWM out so that it occurs during "free time" Currently getting jitter in the PWM signal
2) Get Josh's IR bootloader working...

Bugs:
 - When turned upside down or something the turning goes all weird. Can't seem to recreate this problem...

Analysis:
  - Current draw in sleep is incredibly low, I can't measure it! Current draw with 4 wheels at full speed in the air ~400mA
  - Test whether ESC stuff is working ok and whether it works through the resistors etc.
  - Test RF stuff. No gyro, 3 channel, 2 channel etc.

Use of Peripherals:
  - Timer0 - delay(), millis(), micros()
  - Timer1 - motor drive PWM
  - Timer2 - IR receiving.
  
Description:
  This program is the Code for the Gyro Motor Controller (GMC) board. The board has a number of physical connections:
   - GND and Battery+ power inputs
   - 4 * motor outputs +/-. But there are only 2 motor channels
   - GND and 5v BEC to power a RF receiver if need be
   - 2 * PWM channel inputs from an RF receiver.
   - IR receiver (3.3v, GND, Signal)
   
  The GMC receives from a RF receiver if one is connected otherwise it resorts to receiving from an IR receiver. 
  
  All the GMC really does is act as a gyro stabilised mixer & motor driver for small robots. It uses a PID
  control algorithm running at 100Hz to to keep the robot pointing in the direction stored in the global variable, 
  Yaw_Setpoint. The remote control (IR or RF) changes this variable and the robot attempts to follow. The other input
  is the speed value, which just moves the robot back and forth.
  
  The PID algorithm has a number of tunable parameters that significantly effect the robots character.
  
  - PWM pulsewidths are measured using pinchange interrupts on each channel and the micros() function
  - IR data is read using pinchange interrupts and timer2. It expects manchester encoded data with a bit period of 0.8ms 
   (1 start bit, 8 Yaw_Setpoint bits, 8 Speed bits followed by 4 info bits). It's very robust to spikes and errors. It can
   support 2 different channels by looking at the start bit high time. As long as the both transmitters don't send data literally at the
   same time as each other then this Rx algorithm should latch on and sync with it's own channel.
   
  There is low battery detection (works for 1S or 2S LiPos) and LED indication of various states:
    - Low battery
    - No signal
    - Signal remaining in neutral so idling the motors (otherwise the PID control goes crazy when the robot is picked up...)
    - PID Parameters being updated via IR
*/


#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <EEPROM.h>
#include "Support.h"
#include "Average.h"
#include "IR_Receive.h"

//#define  _REVERSE_MOTOR_POLARITY                                                  // Reverse channels - BIG HERO 6 NEEDS THIS!

//--- IMU6050 Specific: ----------------------------------------------------------
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;                                                            // set true if DMP init was successful
uint8_t mpuIntStatus;                                                             // holds actual interrupt status byte from MPU
uint8_t devStatus;                                                                // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;                                                              // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;                                                               // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];                                                           // FIFO storage buffer

// orientation/motion vars
Quaternion q;                                                                     // [w, x, y, z]         quaternion container
VectorInt16 aa;                                                                   // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;                                                               // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;                                                              // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;                                                              // [x, y, z]            gravity vector
float euler[3];                                                                   // [psi, theta, phi]    Euler angle container
float ypr[3];                                                                     // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;                                               // indicates whether MPU interrupt pin has gone high
 
//--- Globals: -------------------------------------------------------------------
//--- PID Related:                                                                // THESE VALUES ARE READ FROM EEPROM IN THE SETUP ROUTINE. IF THEY HAVEN'T BEEN WRITTEN TO EEPROM YET, BURN THE DEFAULT VALUES IN
float Kp;                                                                         // 1.0    Represents: An error of 90 degrees contributes 90 to the output
float Ki;                                                                         // 1.0e-7 Represents: An error of 90 degrees contributes to 9 to the output per second
float Kd;                                                                         // 1.0e5  Represents: Rotating at 90 degree/second contributes -9 to the output    

float Yaw, Yaw_setpoint;                                                          // The measured Yaw and the desired Yaw provided by the user's stick movements (respectively)

char RF_Receiver_Connected = 0;                                                   // Gets set by calling the IR_OR_PWM() function at start up. 0 for IR, 1 for 2 channel RF, 2 for 3 channel RF.

char IR_Channel = 1;                                                              // Channels 0 or 1 are supported. 
int PWM_Pulse_Width = 0;                                                          // If this is set at any point to be between 800-2000, the GMC starts outputing PWM pulses every 20ms of this length

int FwdBckPulseWdth_Safe;                                                         // The Process_PPM routine saves the volatile versions above to these safe copies for later modification and use  
int LfRghtPulseWdth_Safe;   
int AuxPulseWdth_Safe;

volatile int FwdBckPulseWdth;                                                     // The interrupt routines continously update these whenever a new PPM signal is received (hence volatile!)
volatile int LfRghtPulseWdth;
volatile int AuxPulseWdth;

//--- Class based globals:
VectorFloat Gravity_at_Startup;                                                   // [x, y, z] gravity vector
Average Filter1(ROLLING_AVG_FILTER_LENGTH);
Average Filter2(ROLLING_AVG_FILTER_LENGTH);
IR_Receive IR_Rx;                                                                 // Create instance of the IR_Receive Class which significantly clears up this code.

//--- Routines -------------------------------------------------------------------
void setup_6050(void);                                                            // Initialises the DMP6050
void PID_Routine(void);                                                           // Iterates PID algorithm if enough time has passed since last call and updates the PWM signals to the motors.
void Process_PPM(void);                                                           // Validates and processes PPM signals received by the interrupt routines. Updates Yaw_setpoint. Returns 0 if no PPM signal. 
void Process_IR(void);                                                            // Validates and processes IR signals recieved by the interrupt routines. Updates Yaw_setpoint. Returns 0 if no IR signal.
void IR_OR_PWM(void);                                                             // Discovers whether we have an RF receiver or IR receiver connected at startup. The correct routines get called depending on this
void Update_PID_Values(char Button_Info);                                         // Takes received data over IR and updates the PID values on the fly.
void Access_EEPROM (char Read_Write);                                             // Reads or writes the PID parameters to EEPROM.
char Upside_Down();                                                               // Returns 0 if we're upright, 1 otherwise
void ISR_LR();                                                                    // Interrupt service routine

//--- Setup: ---------------------------------------------------------------------
void setup() 
{  
  // Uncommenting this enables storage of channel in EEPROM which can be annoying...
  //IR_Channel = Get_Channel_From_EEPROM(EEPROM_READ, 0);                           // Get channel we're supposed to on. Can be changed by pressing A, B & Z buttons all at the same time on the GC controller
                                                                                    
  // Read PID values from EEPROM.
  Access_EEPROM(EEPROM_READ);

  // If the values are ridiculous we assume they haven't been written into EEPROM yet so we set them to defaults and burn them in
  if(Kp <= 0.0 || Kp >= 5.0)  {
    Kp = 1.20f; Ki = 2.0e-7f; Kd = 5.0e4f;
    Access_EEPROM(EEPROM_WRITE);Access_EEPROM(EEPROM_WRITE);Access_EEPROM(EEPROM_WRITE);
  }
  if(Ki >= 4.0e-6)  {
    Kp = 1.20f; Ki = 2.0e-7f; Kd = 5.0e4f;
    Access_EEPROM(EEPROM_WRITE);Access_EEPROM(EEPROM_WRITE);Access_EEPROM(EEPROM_WRITE);
  }
  if(Kd >= 2.0e6)  {
    Kp = 1.20f; Ki = 2.0e-7f; Kd = 5.0e4f;
    Access_EEPROM(EEPROM_WRITE);Access_EEPROM(EEPROM_WRITE);Access_EEPROM(EEPROM_WRITE);
  }
    
  // On startup send to the serial port the current PID parameters. There are 10K resistors so no damage will be done if an RF receiver is attached
  Serial.begin(115200);
  delay(250);  
  Serial.print("Kp: ");
  Serial.println(float2s(Kp, 4));
  Serial.print("Ki: ");
  Serial.println(float2s(Ki, 4));
  Serial.print("Kd: ");
  Serial.println(float2s(Kd, 4));
  Serial.end();
  
  setup_6050();
  
  //Setup timer1 Registers for motor PWMing
  TCCR1A = 0b11110001;                                                            // Fast PWM 62.5KHz (8 bit)
  TCCR1B = 0b00001001;
  OCR1A = 128;
  OCR1B = 128;  
  
  //Initialise Pin directions:
  pinMode(LED, OUTPUT);
  
  pinMode(MotorL, OUTPUT);  
  pinMode(MotorR, OUTPUT); 
  
  pinMode(MEnble, OUTPUT);

  pinMode(FwdBckIn, INPUT_PULLUP);
  pinMode(LfRghtIn, INPUT_PULLUP); 

  digitalWrite(IR_Pos, HIGH);  pinMode(IR_Pos, OUTPUT);                           // Power the IR Receiver using the I/Os themselves. The current is small so it's ok.
  digitalWrite(IR_GND, LOW);   pinMode(IR_GND, OUTPUT);
  pinMode(IR_In, INPUT_PULLUP);

  IR_Rx.Configure_Timer2_Interrupts();                                            // Call IR_Rx.Timer_Interrupt() whenever there's a timer2 Compare A match. 

  // Pin change interrupts:  
  attachInterrupt(digitalPinToInterrupt(3), ISR_LR, CHANGE);                      // Attach interrupt on pin INT1        (pin 3)   (FwdBckIn)
  PCMSK0 = 0b00001000;                                                            // Unmask interrupt on PCINT3          (pin 11)  (IR_In)
  PCMSK2 = 0b00000010;                                                            // Unmask interrupt on PCINT17         (pin 1)   (LfRghtIn)  
  PCICR = 0b00000101;                                                             // Enable Interrupts on PCI_2 and PCI_0

  // Detect whether an IR receiver or RF receiver is connected:
  IR_OR_PWM();                                                                    // Sets the RF_Receiver_Connected global variable true if using RF, otherwise 0 (BLOCKS until either IR or RF signal present)
  if(!RF_Receiver_Connected)                                                     
  {
    detachInterrupt(digitalPinToInterrupt(3));                                    // If No RF receiver attached disable the interrupts on these pins and use pin 3 for an ESC PWM signal
    PCMSK2 = 0b00000000; 
    pinMode(PWM_Pin, OUTPUT);
    PWM_Pulse_Width = 0;                                                          // Setting this to a value between 800-2000 enables the PWM output. (At any time)
    
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
        if(RF_Receiver_Connected) {                                               // If an RF receiver is connected take signals from this as it has better performance than an IR controller
          Process_PPM(); 
                                                               
          OCR1A = constrain(128 + FwdBckPulseWdth_Safe + LfRghtPulseWdth_Safe, 0, 255);         
          OCR1B = constrain(128 - FwdBckPulseWdth_Safe + LfRghtPulseWdth_Safe, 0, 255);
        }
        else  {                                                                   // Otherwise we assume an IR receiver is connected. If it's not we shutdown the motors anyway
          Process_IR();                                                   
                      
          OCR1A = constrain(128 + FwdBckPulseWdth_Safe + LfRghtPulseWdth_Safe, 0, 255);         
          OCR1B = constrain(128 - FwdBckPulseWdth_Safe + LfRghtPulseWdth_Safe, 0, 255);             
        } 
      }
    }
  }          

  while (!mpuInterrupt && fifoCount < packetSize) {                               // wait for MPU interrupt or extra packet(s) available          
    if(!Low_Battery())  { 
      if(RF_Receiver_Connected) {                                                 // If an RF receiver is connected take signals from this as it has better performance than an IR controller
        Process_PPM();                                             
        PID_Routine();            
      }
      else  {                                                                     // Otherwise we assume an IR receiver is connected. If it's not we shutdown the motors anyway
        Process_IR();                                             
        PID_Routine();  
      }
    }         
  }             
  
  mpuInterrupt = false;                                                           // reset interrupt flag and get INT_STATUS byte
  mpuIntStatus = mpu.getIntStatus();
  
  fifoCount = mpu.getFIFOCount();                                                 // get current FIFO count
  
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {                               // check for overflow (this should never happen unless our code is too inefficient)      
    mpu.resetFIFO();                                                              // reset so we can continue cleanly      
  }    
  else if (mpuIntStatus & 0x02) {                                                 // otherwise, check for DMP data ready interrupt (this should happen frequently)        
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();                // wait for correct available data length, should be a VERY short wait
      mpu.getFIFOBytes(fifoBuffer, packetSize);                                   // read a packet from FIFO
      
      fifoCount -= packetSize;                                                    // track FIFO count here in case there is > 1 packet available (this lets us immediately read more without waiting for an interrupt)
      
      mpu.dmpGetQuaternion(&q, fifoBuffer);                                       // display Euler angles in degrees
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);        
      Yaw = (ypr[0] * 180.0/M_PI) + 180.0;                                        // Yaw given as a value between 0-360             
  }

  static char Been_Here_Before = 0;
  if(!Been_Here_Before) {                                                         // Get the gravity vector at start up. The robot must be switched on in its UPRIGHT or DOWNRIGHT position
    mpu.dmpGetGravity(&Gravity_at_Startup, &q);
    Been_Here_Before = 1;
  } 
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//#######################################################################################################################################################################
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Process_PPM() 
{  
// A few things about this routine:
//  1) It's run every PPM_UPDATE_PERIOD ms
//  2) It validates PPM data on Ch1 & Ch2 & Ch3 if the latter is present at startup
//  3) Turns LED on to indiciate circuit is receiving valid PPM from receiver     
//  4) If Sticks are in their neutral position for > 1 second, set Yaw_Setpoint = Yaw & stop motors

  // 1) -----  
  static float Zero_Calibration = 0.0;
  static char LfRghtPulse, FwdBckPulse, AuxPulse;
  static int Neutral_Input_Count_PWM = NEUTRAL_THRESHOLD_COUNT; 
  static unsigned long Time_of_last_Update = millis();                            
  static int No_Signal_PWM = NO_SIGNAL_THESHOLD_PWM;                              // Assume no signal at startup
  
  if(millis() - Time_of_last_Update < PPM_UPDATE_PERIOD) { 
    if(No_Signal_PWM > NO_SIGNAL_THESHOLD_PWM || Neutral_Input_Count_PWM >= NEUTRAL_THRESHOLD_COUNT)  {
      DISABLE_MOTORS;
      Yaw_setpoint = Yaw;      
    }   
    return;        
  }  
  Time_of_last_Update += PPM_UPDATE_PERIOD;                                              

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
      
      if(RF_Receiver_Connected != RF_3CH_CTRL)  {
        Yaw_setpoint += (((float)LfRghtPulseWdth_Safe) * 0.05);                   // Scaling so full stick (+500us) gives 500 degrees per second rate of turn
      }
      LfRghtPulseWdth_Safe = LfRghtPulseWdth_Safe / 2;     
      LfRghtPulse = VALID;   
    }
  else  {
      LfRghtPulse = INVALID;
      No_Signal_PWM++;
    }                                                              

//##########################################################################################################
  // Code to allow for 3rd channel RF. Things to consider:
  // Reset the zero value on transmitter on/off cycling.
  // How can drift be adjusted for?

  if(RF_Receiver_Connected == RF_3CH_CTRL)  {
    if((AuxPulseWdth >= 900) && (AuxPulseWdth <= 2100)) {                         // If we have a pulse within valid range
        AuxPulseWdth_Safe = AuxPulseWdth - 1500;
        AuxPulseWdth = 0;
        
        AuxPulseWdth_Safe = AuxPulseWdth_Safe / 2;     
        AuxPulse = VALID;   
      }
    else  {
        AuxPulse = INVALID;
        No_Signal_PWM++;
      } 

    if(FwdBckPulse && LfRghtPulse && AuxPulse)  {                                 // If we've just received 3 valid pulses, clear the No_Signal count    
        No_Signal_PWM = 0;
        
        // Next, overwrite Yaw_setpoint using joytick data and Zero_Calibration, the latter is updated when the transmitter is cycled off and on.
        Yaw_setpoint = Calculate_Joy_Stick(LfRghtPulseWdth_Safe, AuxPulseWdth_Safe) + Zero_Calibration;          
      }        
  }
  else  {
    if(FwdBckPulse && LfRghtPulse)  {                                             // If we've just received 2 valid pulses, clear the No_Signal count    
        No_Signal_PWM = 0;  
      }        
  }    
//##########################################################################################################    
       
  if(Yaw_setpoint >= 360.0)                                                       // Keep Yaw_setpoint within 0-360 limits
      Yaw_setpoint -= 360.0;    
  else if(Yaw_setpoint < 0.0) 
      Yaw_setpoint += 360.0;  

  // 3) -----
  if(No_Signal_PWM > NO_SIGNAL_THESHOLD_PWM)  {     
    LED_OFF;                                                                      // Turn LED off if no signal is being received
    DISABLE_MOTORS;                                                               // Disable outputs  
    PWM_Pulse_Width = 0;                                                          // Stop all signals to ESC
    Zero_Calibration = Yaw_setpoint;                                              // Turning off the signal sets the Zero_Calibration variable
           
    OCR1A = 128; OCR1B = 128; 
    return;
  }   

  // 4) -----
  if(abs(LfRghtPulseWdth_Safe) <= 10 && abs(FwdBckPulseWdth_Safe) <= 10) {        // If control sticks are in their neurtal position...
    Neutral_Input_Count_PWM++;
    
    if(Neutral_Input_Count_PWM >= NEUTRAL_THRESHOLD_COUNT) {
      Yaw_setpoint = Yaw;
      DISABLE_MOTORS;
      
      if(Neutral_Input_Count_PWM % 15 == 0) {                                     // Slow flash LED to indicate the PID algorithm is suspended due to inactivity
        digitalWrite(LED, !digitalRead(LED));      
      }            
      return;                                                                     // Return with a zero to disable motor activity
    }   
  }
  else  {
    Neutral_Input_Count_PWM = 0; 
  }

  ENABLE_MOTORS;
  LED_ON;                                                                         // Turn LED on to indicate signal being received ok and Enable the motors  
  return;  
}

//--------------------------------------------------------------------------------
void Process_IR() 
{ 
// A few things about this routine:
//  1) It's run every IR_UPDATE_PERIOD ms
//  2) It checks if any IR Data is being recieved and whether it's valid.       
//  3) Turns LED on to indiciate circuit is receiving valid IR data from receiver.  
//  4) If the joysticks go still for a while, idle.   

  // 1) ----- 
  static int Neutral_Input_Count_IR = NEUTRAL_THRESHOLD_COUNT;
  static unsigned long Time_of_last_Update = millis(); 
  static int No_Signal_IR = NO_SIGNAL_THESHOLD_IR;                                // Assume no signal at start up
  
  if(millis() - Time_of_last_Update < IR_UPDATE_PERIOD) { 
    if(No_Signal_IR > NO_SIGNAL_THESHOLD_IR || Neutral_Input_Count_IR >= NEUTRAL_THRESHOLD_COUNT)  {
      DISABLE_MOTORS;                                                      
      Yaw_setpoint = Yaw;      
    }        
    return;         
  }  
  Time_of_last_Update += IR_UPDATE_PERIOD; 
      
  if(unsigned long IR_Data = IR_Rx.Check_Data())                                  // If The data contains valid information clear the no signal count and process info
  {                                                               
    No_Signal_IR = 0;
    
    if(IR_Data & 1)     // -----------------------------------------------------   If the packet identifier bit is 1 it's a Type A packet:
    {
      // The common type A packet contains the Yaw_setpoint and Speed data  
    
      Yaw_setpoint = (float)map(((IR_Data >> 9) & 0xFF), 0, 255, 0, 359);
      FwdBckPulseWdth_Safe = (int)((IR_Data >> 1) & 0xFF) - 128;
      FwdBckPulseWdth_Safe = (FwdBckPulseWdth_Safe * 3) / 4; 
       
      static float Old_Yaw_setpoint;
      LfRghtPulseWdth_Safe = round((Yaw_setpoint - Old_Yaw_setpoint) * 5.0);      // In case the Gyro chip isn't populated this still allows for IR robot control
      Old_Yaw_setpoint = Yaw_setpoint;        
    }
    else                // -----------------------------------------------------   If the packet identifier bit is 0 it's a Type B packet:
    {
      // The rarer type B packet contains an 8 bit PWM pulse width value, ..... [Channel change? Bootloader? PID Tuning?] 

      PWM_Pulse_Width = map(((IR_Data >> 9) & 0xFF), 0, 255, 1000, 2000);            
    }
  }
  else  {
    No_Signal_IR++;                                                               // If the data packet was empty which occurs if there was an error etc, increase no signal count          
  }

  
  // 3) -----
  if(No_Signal_IR > NO_SIGNAL_THESHOLD_IR)  {     
    LED_OFF;                                                                      // Turn LED off if no signal is being received
    DISABLE_MOTORS;                                                               // Disable outputs  
    PWM_Pulse_Width = 0;                                                          // Stop all signals to ESC. OR SHOULD WE JUST SET TO 1000?
    Access_EEPROM(EEPROM_WRITE);                                                  // Write PID parameters to EEPROM if they have been updated.

    OCR1A = 128; OCR1B = 128;    
    return;
  }

  // 4) -----
  if(abs(LfRghtPulseWdth_Safe) <= 10 && abs(FwdBckPulseWdth_Safe) <= 10) {        // If control sticks are in their neurtal position...
    Neutral_Input_Count_IR++;
    
    if(Neutral_Input_Count_IR >= NEUTRAL_THRESHOLD_COUNT) {
      Yaw_setpoint = Yaw;
      DISABLE_MOTORS;
      
      if(Neutral_Input_Count_IR % 15 == 0) {                                      // Slow flash LED to indicate the PID algorithm is suspended due to inactivity
        digitalWrite(LED, !digitalRead(LED));      
      }                  
      return;                                                                     // Return with a zero to disable motor activity
    }   
  }
  else  {
    Neutral_Input_Count_IR = 0; 
  }  

  ENABLE_MOTORS;                                                                  
  LED_ON;                                                                         // Turn LED on to indicate signal being received ok and Enable the motors  
  return;  
}

//--------------------------------------------------------------------------------
void PID_Routine()
{
// So we need LfRghtPulseWdth to adjust itself according to how much error there is. Here's a little PID controller to do that. 
// It uses the global variable, Yaw_Setpoint & needs FwdBckPulseWdth_Safe for speed control
// We deduce whether the device has been flipped by comparing the current gravity vector against the startup gravity vector. If their dot product
// is -ve we've been flipped. In this case we need to make just a few alterations to the inputs...

  static unsigned long Time_at_Motor_Update = millis();                           // Want the Function calls to be staggered and this is achieved I think.
  static unsigned long Time_at_PWM_Start;
  
  int FwdBckPulseWdth_Safe_Temp = FwdBckPulseWdth_Safe;
  unsigned long delta_t;

  delta_t = micros() - Time_at_Motor_Update;  
  if(delta_t < 10000)                                                             // Run this routine every 10ms
    return; 
  Time_at_Motor_Update += 10000;

//----------
  if(PWM_Pulse_Width > 800) {                                                     // If a PWM Pulseout is demanded start the pulse here 
    static char Toggle = 0;
    if(Toggle == 0) {                                                             // Only output a pulse every other call to this function: so every 20ms
      Toggle = 1;    

      pinMode(PWM_Pin, OUTPUT);
      PWM_PIN_HIGH;
      Time_at_PWM_Start = micros();                                               // Record the time the Pulseout started
    }
    else {
      Toggle = 0;
    }          
  }

//----------
  if(MOTORS_ENABLED)  
  {
    float Error, Output, dInput;
    static float ErrorSum, LastYaw; 
  
    if(Upside_Down()) {
      float Inverted_Yaw_setpoint = 180.0 - Yaw_setpoint;
      
      while(Inverted_Yaw_setpoint >= 360.0)                                       // Keep Inverted_Yaw_setpoint within 0-360 limits
          Inverted_Yaw_setpoint -= 360.0;    
      while(Inverted_Yaw_setpoint < 0.0) 
          Inverted_Yaw_setpoint += 360.0;    
            
      FwdBckPulseWdth_Safe_Temp = -FwdBckPulseWdth_Safe_Temp;   
      Error = -Turn_Error(Inverted_Yaw_setpoint, Yaw);
      dInput = (LastYaw - Yaw) / delta_t;
    }
    else  {
      Error = Turn_Error(Yaw_setpoint, Yaw); 
      dInput = (Yaw - LastYaw) / delta_t;
    }  
    
    ErrorSum += (Error * Ki) * delta_t;                                           // delta_t ~= 10_000, Also "bumpless"                                                     
    ErrorSum = constrain(ErrorSum, -100.0, 100.0);
    
    LastYaw = Yaw;
    
    Output = (Error * Kp) + ErrorSum - (dInput * Kd);
    
    LfRghtPulseWdth_Safe = round(Output);
    LfRghtPulseWdth_Safe = constrain(LfRghtPulseWdth_Safe, -100, 100);    

    #ifndef _REVERSE_MOTOR_POLARITY         
      OCR1A = Filter1.Rolling_Average(constrain(128 + FwdBckPulseWdth_Safe_Temp + LfRghtPulseWdth_Safe, 0, 255));
      OCR1B = Filter2.Rolling_Average(constrain(128 - FwdBckPulseWdth_Safe_Temp + LfRghtPulseWdth_Safe, 0, 255));
    #else
      OCR1A = Filter1.Rolling_Average(constrain(128 + FwdBckPulseWdth_Safe_Temp + LfRghtPulseWdth_Safe, 0, 255));
      OCR1B = Filter2.Rolling_Average(constrain(128 + FwdBckPulseWdth_Safe_Temp - LfRghtPulseWdth_Safe, 0, 255));
    #endif
  }

  // Blocking code to finish generation of a PWM pulse. This requires the PID algorithm to be called regularly! MUST BE USING IR!   
  if(PWM_PIN_IS_HIGH) {    
    while(micros() - Time_at_PWM_Start < PWM_Pulse_Width) {}
    PWM_PIN_LOW;             
  }   
}

//--------------------------------------------------------------------------------
char Upside_Down()                                                                // Return 0 if we're upright, 1 if we're upside down
{                                                                                 // The dot product goes negative when the current gravity vector is >90 degrees from the startup gravity 
  
  if(((Gravity_at_Startup.x * gravity.x) + (Gravity_at_Startup.y * gravity.y) + (Gravity_at_Startup.z * gravity.z)) > 0)
    return(0);                                                                   
  else
    return(1);                                                                    
}

//--------------------------------------------------------------------------------
void Update_PID_Values(char Button_Info)
{
  if(Button_Info & 0b100)  {                                                      // If Z_Button pressed on GC controller
    if((Button_Info & 0b11) == 0b11)  {                                           // B button
      Kp -= 0.002;
      Kp = max(Kp, 0.0);                                                          // Constrain the min value to 0
      if(Kp > 0.0)                                                                // Don't flash light if we've reached zero
        digitalWrite(LED, !digitalRead(LED));
    }
    if((Button_Info & 0b11) == 0b01)  {                                           // Y button
      Ki -= 8.0e-10;
      Ki = max(Ki, 0.0);
      if(Ki > 0.0)
        digitalWrite(LED, !digitalRead(LED));
    }
    if((Button_Info & 0b11) == 0b10)  {                                           // X button 
      Kd -= 1.0e2;
      Kd = max(Kd, 0.0);
      if(Kd > 0.0)
        digitalWrite(LED, !digitalRead(LED));
    }    
  }
  else  {
    if((Button_Info & 0b11) == 0b11)  {                                           // B button
      Kp += 0.002;
      digitalWrite(LED, !digitalRead(LED));
    }
    if((Button_Info & 0b11) == 0b01)  {                                           // Y button
      Ki += 8.0e-10;
      digitalWrite(LED, !digitalRead(LED));
    }
    if((Button_Info & 0b11) == 0b10)  {                                           // X button 
      Kd += 1.0e2;
      digitalWrite(LED, !digitalRead(LED));
    } 
  }
}

//--------------------------------------------------------------------------------
void Access_EEPROM (char Read_Write)
{
// Bug: When writing new PID values to EEPROM, loosing signal casues a write. The GMC doesn't seem to wake up again...
//      This only happens when 2 or 3 of the parameters have been altered. Something to do with how long eeprom writes take.
//      By Writing one value per call, we get around this problem.

// If Read_Write = 1 this routine writes the current PID parameters to EEPROM otherwise it reads them
  static char i = 0;

  if(Read_Write)
  {
    switch(i) {
      case 0:
        EEPROM.put(0, Kp);
        i = 1;
        return;
      case 1:
        EEPROM.put(4, Ki);
        i = 2;
        return;
      default:
        EEPROM.put(8, Kd);
        i = 0;
        return;
    }
  }
  else
  {
    EEPROM.get(0, Kp);
    EEPROM.get(4, Ki);
    EEPROM.get(8, Kd);    
  }
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

void Sleep_6050() {
  mpu.setSleepEnabled(1);
}

//--------------------------------------------------------------------------------
void IR_OR_PWM()  // RF_Receiver_Connected = 0,1,2,3 - IR connectivity, RF 2 channels, RF 3 channels, RF PPM.   
{
  delay(200);
  
  while(1)                                                                        // Keep looping until either an IR or RF signal is being received since there's no point continuing otherwise!
  {    
    LfRghtPulseWdth = 0;
    FwdBckPulseWdth = 0;
    AuxPulseWdth = 0;

    LED_OFF;
    delay(125);
    LED_ON;
    delay(125);

    if((FwdBckPulseWdth >= 900) && (FwdBckPulseWdth <= 2100)) {                   // If we're receiving RF data return with RF_Receiver_Connected = 1
      if((LfRghtPulseWdth >= 900) && (LfRghtPulseWdth <= 2100)) {
        RF_Receiver_Connected = RF_2CH_CTRL;  
        delay(125);                                                               // Allow time for IR mode to be switched off and for an RF ch3 pulse to be captured.
               
        if((AuxPulseWdth >= 900) && (AuxPulseWdth <= 2100)) {
          RF_Receiver_Connected = RF_3CH_CTRL;      
        }   
        return;
      }
    }
      
    if(IR_Rx.Check_Data())  {                                                     // If we're receiving a proper manchester encoded signal from the IR return with RF_Receiver_Connected = 0;                         
      RF_Receiver_Connected = IR_CTRL;
      return;      
    }        
  } 
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//########## INTERRUPT DETECTION ROUTINES ###############################################################################################################################
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
ISR (PCINT2_vect)                                                                 // Change in LfRghtIn pin of PWM in
{
  static unsigned long Time_at_Rise_FB = 0;
  
  if (LFRGHTIN_STATE == HIGH)
    Time_at_Rise_FB = micros();
  else
    LfRghtPulseWdth = (int)(micros() - Time_at_Rise_FB);
} 

//--------------------------------------------------------------------------------
void ISR_LR()                                                                     // Change in FwdBckIn Pin of PWM in
{
  static unsigned long Time_at_Rise_LR = 0;
  
  if (FWDBCKIN_STATE == HIGH)
    Time_at_Rise_LR = micros();
  else
    FwdBckPulseWdth = (int)(micros() - Time_at_Rise_LR);                                 
}

//--------------------------------------------------------------------------------
void dmpDataReady() 
{
  mpuInterrupt = true;
}

//--------------------------------------------------------------------------------
ISR (TIMER2_COMPA_vect)                                                           // IR dedicated timer compare match     
{  
  IR_Rx.Timer_Interrupt();
} 

//--------------------------------------------------------------------------------
ISR (PCINT0_vect)                                                                 // IR_Pin Change
{ 
  if(!RF_Receiver_Connected)  
  {
    IR_Rx.Pin_Change_Interrupt();
  }
  else  
  {
    static unsigned long Time_at_Rise_Aux = 0;
    
    if (AUXIN_STATE == HIGH)
      Time_at_Rise_Aux = micros();
    else
      AuxPulseWdth = (int)(micros() - Time_at_Rise_Aux);     
  } 
}

