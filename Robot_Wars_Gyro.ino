/*
 * Title: Gyro Motor Controller
 * Author: James Fotherby
 * Date: 2/4/2016
 
Normally we send Type A Packets:
  DataA: Yaw_setpoint & Speed data:  [YYYY_YYYY_SSSS_SSSS_T] - Y = Yaw, S = Speed, T = Packet Type 
  DataB: PWM pulsewidth & info bits: [PPPP_PPPP_BBB_XXONC_T] - P = Pulsewidth, B = PID Tuning, X = Not used
                                                               O = Orientation, N = Use Gyro control, C = Change Channel, T = Packet Type
  
We only send DataB if there has been a change to the PWM_Pulse_Width, or button bits etc. So quite Rarely!
Things to know: Will the ESC work through a 4.7K resistor. Is it ok to loose signal completely or must it be at 1000us when not in use?

To Do:
  - Why is upside down not being detected properly???  
  - Fix the EEPROM Read and write nonsense. Why is it crashing if more than 1 PID value is being written to EEPROM???
      # COULD TRY WRITING TO EEPROM AFTER EVERY PID VALUE UPDATE... 

Analysis:
  - Current draw in sleep is incredibly low, I can't measure it! Current draw with 4 wheels at full speed in the air ~400mA

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
  There are 2 types of RF control: Rate control and Joystick control. Which mode is impliemented depends on whether
  The IR_In pin has a PPM input.
  
  All the GMC really does is act as a gyro stabilised mixer & motor driver for small robots. It uses a PID
  control algorithm running at 50Hz to keep the robot pointing in the direction stored in the global variable, 
  Yaw_Setpoint. The remote control (IR or RF) changes this variable and the robot attempts to follow. The other input
  is the speed value, which just moves the robot back and forth.
  
  The PID algorithm has a number of tunable parameters that significantly effect the robots character.
  
  - PWM pulsewidths are measured using pinchange interrupts on each channel and the micros() function
  - IR data is read using pinchange interrupts and timer2. It expects manchester encoded data with a bit period of 0.8ms 
   (1 start bit, 8 Yaw_Setpoint bits, 8 Speed bits followed by 1 packet type bit). It's very robust to spikes and errors. It can
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

//#define _READ_CHANNEL_FROM_EEPROM                                               // On start up, otherwise default to channel 1

//#define _REVERSE_MOTOR_POLARITY                                                 // Reverse channels - BIG HERO 6 NEEDS THIS!
//#define _GMC_MOUNTED_UPSIDE_DOWN                                                  // Needed if the GMC is mounted with the Z-axis mounted downwards

//############################# VARIABLES ########################################
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
struct PID_Param_Struct {                                                         // THESE VALUES ARE READ FROM EEPROM IN THE SETUP ROUTINE. IF THEY HAVEN'T BEEN WRITTEN TO EEPROM YET, BURN THE DEFAULT VALUES IN
  float Kp;                                                                       // 1.0    Represents: An error of 90 degrees contributes 90 to the output
  float Ki;                                                                       // 1.0e-7 Represents: An error of 90 degrees contributes to 9 to the output per second
  float Kd;                                                                       // 1.0e5  Represents: Rotating at 90 degree/second contributes -9 to the output    
};   

PID_Param_Struct PID_Params;                                                                     

float Yaw, Yaw_setpoint;                                                          // The measured Yaw and the desired Yaw provided by the user's stick movements (respectively)

char Type_of_Reciever = 0;                                                        // Gets set by calling the IR_OR_PWM() function at start up. 0 for IR, 1 for 2 channel RF, 2 for 3 channel RF.
byte Use_Gyro_Flag = 1;                                                           // If this is set to zero, we stop using gyro compensation and act as a normal motor controller
byte Upside_Down_Flag = 0;

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

//############################# ROUTINES #########################################
//--- Routines -------------------------------------------------------------------
void (*boot_bootloader)(void) = (void (*)())0x3c00;                               // Allow the bootloader to be called

void setup_6050(void);                                                            // Initialises the DMP6050
void PID_Routine(void);                                                           // Iterates PID algorithm if enough time has passed since last call and updates the PWM signals to the motors.
void Process_PPM(void);                                                           // Validates and processes PPM signals received by the interrupt routines. Updates Yaw_setpoint. Returns 0 if no PPM signal. 
void Process_IR(void);                                                            // Validates and processes IR signals recieved by the interrupt routines. Updates Yaw_setpoint. Returns 0 if no IR signal.
void IR_OR_PWM(void);                                                             // Discovers whether we have an RF receiver or IR receiver connected at startup. The correct routines get called depending on this
void Update_PID_Values(char Button_Info);                                         // Takes received data over IR and updates the PID values on the fly.
void Access_EEPROM (char Read_Write);                                             // Reads or writes the PID parameters to EEPROM.
char Upside_Down();                                                               // Returns 0 if we're upright, 1 otherwise
void ISR_LR();                                                                    // Interrupt service routine

//--------------------------------------------------------------------------------
//################################################################################
//--------------------------------------------------------------------------------
void setup() 
{  
  // If defined this enables reading of IR channel from EEPROM upon startup
  #ifdef _READ_CHANNEL_FROM_EEPROM
    IR_Rx.Switch_Channel(Get_Channel_From_EEPROM(EEPROM_READ, 0));
  #endif                       
                                                                                    
  // Read PID values from EEPROM.
  Access_EEPROM(EEPROM_READ);

  // If the PID values are ridiculous or haven't been written into EEPROM yet, we set them to defaults and burn them in
  if(PID_Params.Kp <= 0.0 || PID_Params.Kp >= 5.0 || isnan(PID_Params.Kp))  {
    PID_Params.Kp = 0.8f; PID_Params.Ki = 0.5e-7f; PID_Params.Kd = 0.5e5f;
    Access_EEPROM(EEPROM_WRITE);
  }
  if(PID_Params.Ki >= 4.0e-6 || isnan(PID_Params.Ki))  {
    PID_Params.Kp = 0.8f; PID_Params.Ki = 0.5e-7f; PID_Params.Kd = 0.5e5f;
    Access_EEPROM(EEPROM_WRITE);
  }
  if(PID_Params.Kd >= 2.0e6 || isnan(PID_Params.Kd))  {
    PID_Params.Kp = 0.8f; PID_Params.Ki = 0.5e-7f; PID_Params.Kd = 0.5e5f;
    Access_EEPROM(EEPROM_WRITE);
  }
    
  // On startup send to the serial port the current PID parameters. There are 10K resistors so no damage will be done if an RF receiver is attached
  Serial.begin(115200); Serial.println(); 
  Serial.print("Kp: "); Serial.println(float2s(PID_Params.Kp, 4));
  Serial.print("Ki: "); Serial.println(float2s(PID_Params.Ki, 4));
  Serial.print("Kd: "); Serial.println(float2s(PID_Params.Kd, 4));
    
  Serial.print("Channel in EEPROM: ");  
  Serial.println(Get_Channel_From_EEPROM(EEPROM_READ, 0));
  
  Serial.print("Battery voltage (mV): ");  
  Serial.println(analogRead(Vsense) * VOLTAGE_SENSE_CONSTANT);  
  Serial.end();

  // System configuration:
  setup_6050();                                                                   // This configures the DMP6050  
  Configure_Timer1_For_PWM();                                                     // Setup timer1 Registers for motor PWMing   
  Configure_IO_Pins();                                                            // Initialise Pin directions
  IR_Rx.Configure_Timer2_Interrupts();                                            // Call IR_Rx.Timer_Interrupt() whenever there's a timer2 Compare A match. 

  // Configure pin change interrupts:  
  attachInterrupt(digitalPinToInterrupt(3), ISR_LR, CHANGE);                      // Attach interrupt on pin INT1        (pin 3)   (FwdBckIn)
  PCMSK0 = 0b00001000;                                                            // Unmask interrupt on PCINT3          (pin 11)  (IR_In)
  PCMSK2 = 0b00000010;                                                            // Unmask interrupt on PCINT17         (pin 1)   (LfRghtIn)  
  PCICR = 0b00000101;                                                             // Enable Interrupts on PCI_2 and PCI_0

  // Detect whether an IR receiver or RF receiver is connected:
  IR_OR_PWM();                                                                    // BLOCKS until either IR or RF signal present
}

//--------------------------------------------------------------------------------
//################################################################################
//--------------------------------------------------------------------------------
void loop() 
{
  while (!mpuInterrupt && fifoCount < packetSize) {                               // wait for MPU interrupt or extra packet(s) available          
    if(!Low_Battery())  { 
      if(Type_of_Reciever == RF_CTRL)                                             // If an RF receiver is connected take signals from this as it has better performance than an IR controller
        Process_PPM(); 
      else                                                                        // Otherwise we assume an IR receiver is connected. If it's not we shutdown the motors anyway
        Process_IR();                                           
      
      PID_Routine();
    }         
  }  

  Update_Yaw();
}

//--------------------------------------------------------------------------------
//################################################################################
//--------------------------------------------------------------------------------
void Process_PPM() 
{  
// A few things about this routine:
//  1) It's run every PPM_UPDATE_PERIOD ms
//  2) It validates PPM data on Ch1 & Ch2 & Ch3
//  3) Turns LED on to indiciate circuit is receiving valid PPM from receiver, off otherwise.     
//  4) If Sticks are in their neutral position for > 1 second, enter an idle mode where motors are disabled and LED blinks
    
  static float Zero_Calibration = 0.0;
  static byte LfRghtPulse, FwdBckPulse, AuxPulse;
  static int Neutral_Input_Count_PWM = NEUTRAL_THRESHOLD_COUNT; 
  static unsigned long Run_PPM_Update = millis() + PPM_UPDATE_PERIOD;                            
  static int No_PWM_Signal = NO_PWM_SIGNAL_THESHOLD;                              // Assume no signal at startup

  // 1)----******-----******-----******-----******-----******-----******-----*****
  if(millis() <= Run_PPM_Update)
    return;         
  
  Run_PPM_Update += PPM_UPDATE_PERIOD;                                              

  unsigned long Min_Time_Delay = millis() + PPM_UPDATE_PERIOD - 10;               // Constrain max and min times to next run this routine
  unsigned long Max_Time_Delay = millis() + PPM_UPDATE_PERIOD + 10;
  Run_PPM_Update = constrain(Run_PPM_Update, Min_Time_Delay, Max_Time_Delay); 

  // 2)----******-----******-----******-----******-----******-----******-----*****
  if((FwdBckPulseWdth >= 900) && (FwdBckPulseWdth <= 2100)) {                     // If we have a pulse within valid range
      FwdBckPulseWdth_Safe = FwdBckPulseWdth - 1500;
      FwdBckPulseWdth = 0;                                                        // If no new values are received this ensures we don't keep using old PWM data
      FwdBckPulse = VALID;                  
    }
  else  {  
      FwdBckPulse = INVALID;
      No_PWM_Signal++;        
    }
  
  if((LfRghtPulseWdth >= 900) && (LfRghtPulseWdth <= 2100)) {                     // If we have a pulse within valid range
      LfRghtPulseWdth_Safe = LfRghtPulseWdth - 1500;
      LfRghtPulseWdth = 0;
      LfRghtPulse = VALID;   
    }
  else  {
      LfRghtPulse = INVALID;
      No_PWM_Signal++;
    }

  if((AuxPulseWdth >= 900) && (AuxPulseWdth <= 2100)) {                           // If we have a pulse within valid range
      AuxPulseWdth_Safe = AuxPulseWdth - 1500;
      AuxPulseWdth = 0;        
      AuxPulse = VALID;   
    }
  else  {
      AuxPulse = INVALID;
      No_PWM_Signal++;
    }
   
//########################################################################################################## 
// Here we decide how to use the PPM data:
// 1) If there are 3 channels connected use Joystick Control with separate throttle provided by the 3rd channel
// 2) If only 2 channels connected and AuxPWMIn is pulled low use Joystick Control with throttle proportional to deviation of stick - you can't reverse, feels weird
// 3) If only 2 channels connected and AuxPWMIn is left to float high then resort to rate control - ie. throttle and left/right but with gyro compensation

  if(FwdBckPulse && LfRghtPulse)  {                                               // If we've just received 2 valid pulses, clear the No_Signal count    
    //PID_Tuning(No_PWM_Signal);                                                    // PID Tuning if Tx turned on with sticks in bottom right position.
    No_PWM_Signal = 0;

    if(AuxPulse)                                                                  // If we're receiving PPM on Aux_In: Joystick Control with separate throttle...
    {
      float Joystick_Sq_Mag = Calculate_Joy_Stick_Magnitude(FwdBckPulseWdth_Safe, LfRghtPulseWdth_Safe); 
      if(Joystick_Sq_Mag > 20000.0)                                               // Require a stick deviation of ~25% before using it to set Yaw_Setpoint, otherwise erratic.
      {
        Yaw_setpoint = Calculate_Joy_Stick_Angle(FwdBckPulseWdth_Safe, LfRghtPulseWdth_Safe) + Zero_Calibration;
      }    
      
      FwdBckPulseWdth_Safe = AuxPulseWdth_Safe / 4;
      Use_Gyro_Flag = 1;     
    }
    else if(!AUXIN_STATE)                                                         // If AuxPWMIn is pulled low, don't use gyro control
    {         
      FwdBckPulseWdth_Safe /= 4;  
      LfRghtPulseWdth_Safe /= 6;
      Use_Gyro_Flag = 0;
    }
    else                                                                          // If AuxPWMIn is disconnected and therefore pulled high default to rate control...
    {
      Yaw_setpoint += (((float)LfRghtPulseWdth_Safe) * 0.05);                     // Scaling so full stick (+500us) gives 500 degrees per second rate of turn
      FwdBckPulseWdth_Safe /= 4;   
      Use_Gyro_Flag = 1;   
    }
  }      
      
//##########################################################################################################    
  Yaw_setpoint = Circularly_Constrain(Yaw_setpoint);                 // Keep Yaw_setpoint within 0-360 limits

  // 3)----******-----******-----******-----******-----******-----******-----*****  
  if(No_PWM_Signal > NO_PWM_SIGNAL_THESHOLD)  {     
    LED_OFF;                                                                      // Turn LED off if no signal is being received
    DISABLE_MOTORS;                                                               // Disable outputs  
    Zero_Calibration = Yaw_Setpoint;                                              // Turning off the signal sets the Zero_Calibration variable
           
    OCR1A = 128; OCR1B = 128;                                                 
    return;
  }   

  // 4) -----                                                                     // If control sticks are in their neurtal position...
  if(abs(LfRghtPulseWdth_Safe) <= 20 && abs(FwdBckPulseWdth_Safe) <= 10 && Use_Gyro_Flag) {        
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
//################################################################################
//--------------------------------------------------------------------------------
void Process_IR() 
{ 
// A few things about this routine:
//  1) It's run every IR_UPDATE_PERIOD ms
//  2) It checks if any IR Data is being recieved and whether it's valid.       
//  3) Turns LED on to indiciate circuit is receiving valid IR data from receiver.  
  
  static unsigned long Run_IR_Update = millis() + IR_UPDATE_PERIOD; 
  static int No_IR_Signal = NO_IR_SIGNAL_THESHOLD;                                // Assume no signal at start up
  static int Idle_Flag = 0;

  // 1)----******-----******-----******-----******-----******-----******-----***** 
  if(millis() <= Run_IR_Update)       
    return;         
    
  Run_IR_Update += IR_UPDATE_PERIOD;

  unsigned long Min_Time_Delay = millis() + IR_UPDATE_PERIOD - 10;                // Constrain max and min times to next run this routine
  unsigned long Max_Time_Delay = millis() + IR_UPDATE_PERIOD + 10;
  Run_IR_Update = constrain(Run_IR_Update, Min_Time_Delay, Max_Time_Delay); 
 
  // 2)----******-----******-----******-----******-----******-----******-----*****    
  if(unsigned long IR_Data = IR_Rx.Check_Data())                                  // If data contains valid information clear the no signal count and process info
  {                                                               
    No_IR_Signal = 0;

    // If the packet identifier bit is 1 it's a Type A packet: 
    if(IR_Data & 1)     
    {
      // Type A packets contain: Yaw_setpoint & Speed data: [YYYY_YYYY__SSSS_SSSS_T] - Y = Yaw, S = Speed, T = Packet Type    
      FwdBckPulseWdth_Safe = (int)((IR_Data >> 1) & 0xFF) - 128;
            
      if(Use_Gyro_Flag) {                
        Yaw_setpoint = (float)map(((IR_Data >> 9) & 0xFF), 0, 255, 0, 359);
        
        if(Yaw_setpoint == 0)
          Idle_Flag = 1;
        else
          Idle_Flag = 0;
      }
      else  {
        LfRghtPulseWdth_Safe = (int)((IR_Data >> 9) & 0xFF) - 128; 
      }   
    }

    // If the packet identifier bit is 0 it's a Type B packet:
    else                
    {
      // Type B packets contain: PWM pulsewidth & info bits: [PPPP_PPPP_BBB_XXONCT] - P = Pulsewidth, B = PID Tuning, X = Not used
      PWM_Pulse_Width = map(((IR_Data >> 9) & 0xFF), 0, 255, 1000, 2000);        // - O = Orientation, N = Use Gyro control, C = Change Channel, T = Packet Type
      
      if(unsigned char Button_Bits = ((IR_Data >> 6) & 0x7)) {
        Update_PID_Values(Button_Bits);
      } 

      if((IR_Data >> 3) & 1)                                                     // Since I can't get the IMU to detect if we're flipped or not, we have to send a signal
        Upside_Down_Flag = 0;
      else
        Upside_Down_Flag = 1;

      if((IR_Data >> 2) & 1)                                                     // Stops Gyro compensation and runs motors like a normal motor controller
        Use_Gyro_Flag = 0;
      else
        Use_Gyro_Flag = 1;
      
      if((IR_Data >> 1) & 1)  {                                                   // This toggles the IR channel that we're using and stores the new value in EEPROM
        byte Channel_To_Switch_To = !Get_Channel_From_EEPROM(EEPROM_READ, 0);                  
        IR_Rx.Switch_Channel(Channel_To_Switch_To);                                            
        Get_Channel_From_EEPROM(EEPROM_WRITE, Channel_To_Switch_To); 
      }      
    }
  }
  else  {
    No_IR_Signal++;                                                               // If the data packet was empty which occurs if there was an error etc, increase no signal count          
  }
  
  // 3)----******-----******-----******-----******-----******-----******-----*****
  if(No_IR_Signal == NO_IR_SIGNAL_THESHOLD) {
    DISABLE_MOTORS;
    Access_EEPROM(EEPROM_WRITE);    
  }  
  else if(No_IR_Signal > NO_IR_SIGNAL_THESHOLD)  {     
    LED_OFF;                                                                      // Turn LED off if no signal is being received
    DISABLE_MOTORS;                                                               // Disable outputs  
    PWM_Pulse_Width = 1000;                                                       // Power down ESC.

    OCR1A = 128; OCR1B = 128;    
    return;
  }

  if(Idle_Flag) {                                                                 // If we're being told to idle by the Tx, disable motors and slow flash LED
    DISABLE_MOTORS;    
    static byte Idle_Count = 0; Idle_Count++;
    if(Idle_Count % 15 == 0) { digitalWrite(LED, !digitalRead(LED)); }            // Slow flash LED to indicate we're idling  
    return;
  }    

  ENABLE_MOTORS;                                                                  
  LED_ON;                                                                         // Turn LED on to indicate signal being received ok and Enable the motors  
  return;  
}

//--------------------------------------------------------------------------------
//################################################################################
//--------------------------------------------------------------------------------
void PID_Routine()
{
// So we need the robot to try to turn itself so its measured Yaw becomes the same as the demanded Yaw_Setpoint. Here's a little PID controller to do that. 
// Tuning of the PID parameters: Kp, Ki, & Kd massively effect the behaviour of the robot. Tuning on the fly can be done.
// We deduce whether the device has been flipped by comparing the current gravity vector against the startup gravity vector. If their dot product
// is -ve we've been flipped. In this case we need to make just a few alterations to the inputs...

  static unsigned long Time_at_Motor_Update = micros();
  static unsigned long Run_Next_PID = micros() + PID_UPDATE_PERIOD;                           
  static unsigned long Time_at_PWM_Start;
  
  int FwdBckPulseWdth_Safe_Temp = FwdBckPulseWdth_Safe;
  int LfRghtPulseWdth_Safe_Temp = LfRghtPulseWdth_Safe;
  unsigned long delta_t;
    
  if(micros() < Run_Next_PID)                                                     // Run this routine every 20ms
    return;

  //------------------------------------------------------------------------------
  if(Type_of_Reciever == IR_CTRL)  
  { 
    if(PWM_Pulse_Width > 800) {                                                   // If a PWM Pulseout is demanded start the pulse here 
      pinMode(PWM_Pin, OUTPUT);
      PWM_PIN_HIGH;
      Time_at_PWM_Start = micros();                                               // Record the time the Pulseout started           
    }
  
    //--- Code to keep PID_Updates synchronised to lie outside packet receives (0-14.4ms). So perform PID_Updates at 17ms and 37ms within our 40ms period
    unsigned long Start_Time_of_Last_Fully_Received_Packet = IR_Rx.Packet_Start_Time();
    unsigned long Time_Since_Start_of_Last_Fully_Received_Packet = micros() - Start_Time_of_Last_Fully_Received_Packet;
    
    if(Time_Since_Start_of_Last_Fully_Received_Packet <= 40000) {                 // True if we're not receiving a packet right now but one has recently finished being received (< 24.6ms ago).                                                                       
      if(Run_Next_PID - Start_Time_of_Last_Fully_Received_Packet > 20000) {       // True only for the 37ms pulse
        if(Run_Next_PID - Start_Time_of_Last_Fully_Received_Packet < 37000)       // This should be the 37ms PID_Update but it's a bit early 
          Run_Next_PID += 300;                                                    // So Shift all PID_Updates a bit later 
        else if(Run_Next_PID - Start_Time_of_Last_Fully_Received_Packet > 38000)  // This should be the 37ms PID_Update but it's a bit late
          Run_Next_PID -= 300;                                                    // So Shift all PID_Updates a bit earlier 
      }
    }
  }
  //------------------------------------------------------------------------------
  Run_Next_PID += PID_UPDATE_PERIOD;

  unsigned long Min_Time_Delay = millis() + PID_UPDATE_PERIOD - 5000;             // Constrain max and min times to next run this routine
  unsigned long Max_Time_Delay = millis() + PID_UPDATE_PERIOD + 5000;
  Run_Next_PID = constrain(Run_Next_PID, Min_Time_Delay, Max_Time_Delay);
                                                                
  //------------------------------------------------------------------------------
  if(MOTORS_ENABLED) {
    if(Use_Gyro_Flag)  
    {
      delta_t = micros() - Time_at_Motor_Update;  
      Time_at_Motor_Update = micros();
      
      float Error, Output, dInput;
      static float ErrorSum, LastYaw;

      if(Upside_Down()) 
      {
        float Inverted_Yaw_setpoint = 180.0 - Yaw_setpoint;
        Inverted_Yaw_setpoint = Circularly_Constrain(Inverted_Yaw_setpoint);      // Keep Inverted_Yaw_setpoint within 0-360 limits        
              
        FwdBckPulseWdth_Safe_Temp = -FwdBckPulseWdth_Safe_Temp;   
        Error = -Turn_Error(Inverted_Yaw_setpoint, Yaw);
        dInput = (LastYaw - Yaw) / delta_t;
      }
      else  
      {
        Error = Turn_Error(Yaw_setpoint, Yaw); 
        dInput = (Yaw - LastYaw) / delta_t;
      }  
      
      ErrorSum += (Error * PID_Params.Ki) * delta_t;                                                                                               
      ErrorSum = constrain(ErrorSum, -100.0, 100.0);      
      LastYaw = Yaw;
  
      #ifdef _GMC_MOUNTED_UPSIDE_DOWN
        Output = (Error * PID_Params.Kp) + ErrorSum + (dInput * PID_Params.Kd);
      #else
        Output = (Error * PID_Params.Kp) + ErrorSum - (dInput * PID_Params.Kd);
      #endif
      
      LfRghtPulseWdth_Safe_Temp = round(Output);
      LfRghtPulseWdth_Safe_Temp = constrain(LfRghtPulseWdth_Safe_Temp, -100, 100);
    }    
    else
    { 
      if(Upside_Down()) {             
        FwdBckPulseWdth_Safe_Temp = -FwdBckPulseWdth_Safe_Temp;
        LfRghtPulseWdth_Safe_Temp = -LfRghtPulseWdth_Safe_Temp;  
      }
    }
      
    #if defined(_REVERSE_MOTOR_POLARITY)
      OCR1A = Filter1.Rolling_Average(constrain(128 + FwdBckPulseWdth_Safe_Temp + LfRghtPulseWdth_Safe_Temp, DUTY_MIN, DUTY_MAX));
      OCR1B = Filter2.Rolling_Average(constrain(128 + FwdBckPulseWdth_Safe_Temp - LfRghtPulseWdth_Safe_Temp, DUTY_MIN, DUTY_MAX));
    #elif defined(_GMC_MOUNTED_UPSIDE_DOWN)
      OCR1A = Filter1.Rolling_Average(constrain(128 - FwdBckPulseWdth_Safe_Temp + LfRghtPulseWdth_Safe_Temp, DUTY_MIN, DUTY_MAX));
      OCR1B = Filter2.Rolling_Average(constrain(128 + FwdBckPulseWdth_Safe_Temp + LfRghtPulseWdth_Safe_Temp, DUTY_MIN, DUTY_MAX));    
    #else
      OCR1A = Filter1.Rolling_Average(constrain(128 + FwdBckPulseWdth_Safe_Temp + LfRghtPulseWdth_Safe_Temp, DUTY_MIN, DUTY_MAX));
      OCR1B = Filter2.Rolling_Average(constrain(128 - FwdBckPulseWdth_Safe_Temp + LfRghtPulseWdth_Safe_Temp, DUTY_MIN, DUTY_MAX));
    #endif
  }

  // Blocking code to finish generation of a PWM pulse. This requires the PID algorithm to be called regularly! Stops working should Low Battery occur.                                             
  if(PWM_Pulse_Width) {    
    while(micros() - Time_at_PWM_Start < PWM_Pulse_Width) {}
    PWM_PIN_LOW;                                                                 
  }     
}

//--------------------------------------------------------------------------------
//################################################################################
//--------------------------------------------------------------------------------
char Upside_Down()                                                                // Return 0 if we're upright, 1 if we're upside down
{ 
  //return(Upside_Down_Flag); //########################## Temp Fix ##############################
                                                                                   
  if(((Gravity_at_Startup.x * gravity.x) + (Gravity_at_Startup.y * gravity.y) + (Gravity_at_Startup.z * gravity.z)) > 0)  {
    #ifdef _GMC_MOUNTED_UPSIDE_DOWN
      return(1);
    #else
      return(0);
    #endif 
  }                                                                  
  else  {
    #ifdef _GMC_MOUNTED_UPSIDE_DOWN
      return(0);
    #else
      return(1);
    #endif         
  }                                                          
}

//--------------------------------------------------------------------------------
//################################################################################
//--------------------------------------------------------------------------------
void Update_PID_Values(char Button_Info)
{
  if(Button_Info & 0b100)  {                                                      // If Z_Button pressed on GC controller
    if((Button_Info & 0b11) == 0b11)  {                                           // B button
      PID_Params.Kp -= 0.004;
      PID_Params.Kp = max(PID_Params.Kp, 0.0);                                    // Constrain the min value to 0
      if(PID_Params.Kp > 0.0)                                                     // Don't flash light if we've reached zero
        digitalWrite(LED, !digitalRead(LED));
    }
    if((Button_Info & 0b11) == 0b01)  {                                           // Y button
      PID_Params.Ki -= 1.6e-9;
      PID_Params.Ki = max(PID_Params.Ki, 0.0);
      if(PID_Params.Ki > 0.0)
        digitalWrite(LED, !digitalRead(LED));
    }
    if((Button_Info & 0b11) == 0b10)  {                                           // X button 
      PID_Params.Kd -= 2.0e2;
      PID_Params.Kd = max(PID_Params.Kd, 0.0);
      if(PID_Params.Kd > 0.0)
        digitalWrite(LED, !digitalRead(LED));
    }    
  }
  else  {
    if((Button_Info & 0b11) == 0b11)  {                                           // B button
      PID_Params.Kp += 0.004;
      digitalWrite(LED, !digitalRead(LED));
    }
    if((Button_Info & 0b11) == 0b01)  {                                           // Y button
      PID_Params.Ki += 1.6e-9;
      digitalWrite(LED, !digitalRead(LED));
    }
    if((Button_Info & 0b11) == 0b10)  {                                           // X button 
      PID_Params.Kd += 2.0e2;
      digitalWrite(LED, !digitalRead(LED));
    } 
  }
}

//--------------------------------------------------------------------------------
//################################################################################
//--------------------------------------------------------------------------------
void Access_EEPROM (char Read_Write)
{
  if(Read_Write)  {                                                               // If Read_Write = 1 this routine writes the current PID parameters to EEPROM otherwise it reads them
    EEPROM.put(12, PID_Params);
  }
  else  {
    EEPROM.get(12, PID_Params);   
  }
}

//--------------------------------------------------------------------------------
//################################################################################
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

    if (!dmpReady)  { 
      Beep_Motors(4000, 50); Beep_Motors(3000, 50); Beep_Motors(4000, 50);        // Beep to indicate Gyro isn't working
      Beep_Motors(3000, 50); Beep_Motors(4000, 50); Beep_Motors(3000, 50);
      Use_Gyro_Flag = 0;                                                          // Ensure we don't try to use the gyro since it isn't working
    }       
}

//--------------------------------------------------------------------------------
//################################################################################
//--------------------------------------------------------------------------------
void Sleep_6050() 
{
  mpu.setSleepEnabled(1);
}

//--------------------------------------------------------------------------------
//################################################################################
//--------------------------------------------------------------------------------
void Update_Yaw()
{
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

  // This grabs the gravity vector at startup so that we can later see if the robot has been flipped or not. 
  static char Gravity_Aquired = 0;
  static char Get_Gravity_Countdown = 25;                                         // Get gravity vector after the 25th iteration to this routine
  
  if(!Gravity_Aquired) {                                                          // Get the gravity vector at start up. The robot must be switched on in its UPRIGHT position
    if(Get_Gravity_Countdown) {
      Get_Gravity_Countdown--;
    }
    else  {
      Gravity_at_Startup = gravity;
      Gravity_Aquired = 1;            
    }
  }
}

//--------------------------------------------------------------------------------
//################################################################################
//--------------------------------------------------------------------------------
void IR_OR_PWM()                                                                  // Type_of_Reciever = 0,1,2,3 - IR connectivity, RF 2 channels, RF 3 channels, RF PPM.   
{
  delay(100);
  
  while(1)                                                                        // Keep looping until either an IR or RF signal is being received since there's no point continuing otherwise!
  {    
    LfRghtPulseWdth = 0; FwdBckPulseWdth = 0; AuxPulseWdth = 0;   
    
    LED_ON;
    for(int i = 0; i < 10; i++) {                                                 // Delays 200ms and allows battery sensing to be performed etc
      Low_Battery();
      delay(20);
    } 

    // Here we check whether an PPM signal is being received:
    if((FwdBckPulseWdth >= 900) && (FwdBckPulseWdth <= 2100)) {                   
      if((LfRghtPulseWdth >= 900) && (LfRghtPulseWdth <= 2100)) {
        Type_of_Reciever = RF_CTRL;                                               // IR mode is switched off allowing for possible RF ch3 pulses to be captured.   
        break;
      }
    }

    // Here we check whether an IR signal is being received:
    if(IR_Rx.Check_Data())  {                                                     // If we're receiving a proper manchester encoded signal from the IR return with Type_of_Reciever = IR_CTRL                         
      Type_of_Reciever = IR_CTRL;
      break;      
    }  

    LED_OFF;    
    for(int i = 0; i < 10; i++) {                                                 
      Low_Battery();
      delay(20);
    }           
  } 

  if(Type_of_Reciever == IR_CTRL)                                                     
  {
    detachInterrupt(digitalPinToInterrupt(3));                                    // If No RF receiver attached disable the interrupts on these pins and use pin 3 for an ESC PWM signal
    PCMSK2 = 0b00000000; 
    
    pinMode(PWM_Pin, OUTPUT);
    PWM_Pulse_Width = 1000;                                                       // Setting this to a value between 800-2000 enables the PWM output
    
    Beep_Motors(4000, 100);                                                       // Beep twice if IR being used, High then Low pitch
    delay(100);
  }
  else                                                                            // Beep once if RF ctrl has been detected
  {
    pinMode(IR_Pos, INPUT);
  }

  Beep_Motors(3000, 100);  
}

//--------------------------------------------------------------------------------
//################################################################################
//--------------------------------------------------------------------------------
void PID_Tuning(int No_PWM_Signal)
{
  if(No_PWM_Signal < NO_PWM_SIGNAL_THESHOLD)                                      // If Tx hasn't just been turned on, return immediately
    return;

  delay(50);                                                                      // ?need? - When Tx turns on, does it need time to output sensible PWM pulses?
  if(FwdBckPulseWdth_Safe < 300 && LfRghtPulseWdth_Safe < 300)                    // If stick isn't in top right postion then don't enter tuning routine
    return;

  Beep_Motors(4000, 50); Beep_Motors(3000, 50);                                   // Beep to indicate we're in the tuning routine 

  unsigned long Last_Stick_Movement = millis();
  while(millis() - Last_Stick_Movement < 2000)                                    // If stick stays in center for >2seconds, exit this routine
  {
    if(FwdBckPulseWdth_Safe > 300 && LfRghtPulseWdth_Safe > 300)  {               // Increase P if stick in top right
      Update_PID_Values(0b011);
      Last_Stick_Movement = millis();
    }  
    else if(FwdBckPulseWdth_Safe > 300 && LfRghtPulseWdth_Safe < -300)  {         // Decrease P if stick in top left
      Update_PID_Values(0b111);
      Last_Stick_Movement = millis(); 
    }          
    else if(LfRghtPulseWdth_Safe > 300) {                                         // Increase I if stick in middle right
      Update_PID_Values(0b001);
      Last_Stick_Movement = millis();
    }      
    else if(LfRghtPulseWdth_Safe < -300)  {                                       // Decrease I if stick in middle left
      Update_PID_Values(0b101);
      Last_Stick_Movement = millis(); 
    }         
    else if(FwdBckPulseWdth_Safe < -300 && LfRghtPulseWdth_Safe > 300)  {         // Increase D if stick in bottom right
      Update_PID_Values(0b010);
      Last_Stick_Movement = millis(); 
    }         
    else if(FwdBckPulseWdth_Safe < -300 && LfRghtPulseWdth_Safe < -300) {         // Decrease I if stick in bottom left
      Update_PID_Values(0b110);
      Last_Stick_Movement = millis();
    } 

    delay(40);
  }  
}






//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//########## INTERRUPT DETECTION ROUTINES ###############################################################################################################################
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
ISR (PCINT2_vect)                                                                 // Change in LfRghtIn pin of PWM in
{
  static unsigned long Time_at_Rise_FB = 0;
  
  if (LFRGHTIN_STATE)
    Time_at_Rise_FB = micros();
  else
    LfRghtPulseWdth = (int)(micros() - Time_at_Rise_FB);
} 

//--------------------------------------------------------------------------------
void ISR_LR()                                                                     // Change in FwdBckIn Pin of PWM in
{
  static unsigned long Time_at_Rise_LR = 0;
  
  if (FWDBCKIN_STATE)
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
  if(Type_of_Reciever == IR_CTRL)  
  {
    IR_Rx.Pin_Change_Interrupt();
  }
  else  
  {
    static unsigned long Time_at_Rise_Aux = 0;
    
    if (AUXIN_STATE)
      Time_at_Rise_Aux = micros();
    else
      AuxPulseWdth = (int)(micros() - Time_at_Rise_Aux);     
  } 
}

