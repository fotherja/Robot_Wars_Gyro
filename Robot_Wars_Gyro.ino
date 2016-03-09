/*
 * 
To Do:
1) Allow for single wire PPM input?
6) Could add a moving avergae filter to the motors to reduce such harsh accelerations...

SOME BIG IMPROVMENETSS
1) Just whack a blocking PWM generation pulse after every other call of the PID algorithm. <- DONE!
2) Carry out IR_Process every 41ms exactly rather than 41ms after it passes. <- DONE
3) After detection of initial start pulse, don't search again until 38ms later. ie. until just before packet is expected again. This will allow ignorance of inbetween 
   data being sent to another robot. Would almost get away with same start bits. But no have a 200us and 600us start bit for channel 0 and 1 respectively. This is exactly what we want!!!! <- DONE


5) yeah I think it might be better if motors don't switch off every failed process_IR. this might be the reason for the current jumpyness <- DONE
6) Direct register writes for pin changes/reads etc

How are we doing on CPU usage?



Use of Peripherals:
  - Timer0 - delay(), millis(), micros()
  - Timer1 - motor drive PWM
  - Timer2 - IR receiving and ESC PWM output.
  
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
   (1 start bit, 8 Yaw_Setpoint bits, 8 Speed bits followed by 4 info bits). It's very robust to spikes and errors.
   
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

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
 
//--- Globals: -------------------------------------------------------------------
//--- PID Related:                                                                // THESE VALUES ARE READ FROM EEPROM IN THE SETUP ROUTINE. IF THEY HAVEN'T BEEN WRITTEN TO EEPROM YET, BURN THE DEFAULT VALUES IN
float Kp;                                                                         // 1.0    Represents: An error of 90 degrees contributes 90 to the output
float Ki;                                                                         // 1.0e-7 Represents: An error of 90 degrees contributes to 9 to the output per second
float Kd;                                                                         // 1.0e5  Represents: Rotating at 90 degree/second contributes -9 to the output    

volatile long FwdBckPulseWdth;                                                    // The interrupt routines continously update these whenever a new PPM signal is received (hence volatile!)
volatile long LfRghtPulseWdth;

long FwdBckPulseWdth_Safe;                                                        // The Process_PPM routine saves the volatile versions above to these safe copies for later modification and use  
long LfRghtPulseWdth_Safe;

float Yaw, Yaw_setpoint;                                                          // The measured Yaw and the desired Yaw provided by the user's stick movements (respectively)

char RF_Receiver_Connected;

int PWM_Pulse_Width = 0;                                                          // If this is set at any point to be between 800-2000, the GMC starts outputing PWM pulses every 20ms of this length
VectorFloat Gravity_at_Startup;                                                   // [x, y, z] gravity vector

//--- IR Related:
volatile unsigned long  Total_Time_High;                                          // Measures the time spent high during a bit_period. If Total_Time_High > 1/2 * Bit period we assume a 1 was sent. Filters spikes
volatile unsigned long  TimeA, Time_Diff;
volatile unsigned long  Encoded_Data[2];

volatile boolean        Decode_Flag;
volatile boolean        State;
volatile int            bit_index = 0;

//--- Routines -------------------------------------------------------------------
void setup_6050(void);                                                            // Initialises the DMP6050
char PID_Routine(void);                                                           // Iterates PID algorithm if enough time has passed since last call and updates the PWM signals to the motors. Returns 1 if update performed
char Process_PPM(void);                                                            // Validates and processes PPM signals received by the interrupt routines. Updates Yaw_setpoint. Returns 0 if no PPM signal. 
char Process_IR(void);                                                             // Validates and processes IR signals recieved by the interrupt routines. Updates Yaw_setpoint. Returns 0 if no IR signal.
long Decode(void);                                                                // Decodes the Manchester encoded received IR Data
void IR_OR_PWM(void);                                                             // Discovers whether we have an RF receiver or IR receiver connected at startup. The correct routines get called depending on this
void Update_PID_Values(char Button_Info);                                         // Takes received data over IR and updates the PID values on the fly.
void Access_EEPROM (char Read_Write);                                             // Reads or writes the PID parameters to EEPROM.
char Upside_Down();                                                               // Returns 0 if we're upright, 1 otherwise

//--- Setup: ---------------------------------------------------------------------
void setup() 
{  
  // Read PID values from EEPROM.
  Access_EEPROM(0);

  // If the values are ridiculous we assume they haven't been written into EEPROM yet so we set them to defaults and burn them in
  if(Kp <= 0.0 || Kp >= 5.0)  {
    Kp = 1.20f; Ki = 2.0e-7f; Kd = 5.0e4f;
    Access_EEPROM(1);
  }
  if(Ki >= 4.0e-6)  {
    Kp = 1.00f; Ki = 4.0e-7f; Kd = 5.0e4f;
    Access_EEPROM(1);
  }
  if(Kd >= 2.0e6)  {
    Kp = 1.00f; Ki = 4.0e-7f; Kd = 5.0e4f;
    Access_EEPROM(1);
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

  digitalWrite(IR_Pos, HIGH);  pinMode(IR_Pos, OUTPUT);                           // Power the IR Reciever using the I/Os themselves. The current is small so it's ok.
  digitalWrite(IR_GND, LOW);   pinMode(IR_GND, OUTPUT);
  pinMode(IR_In, INPUT_PULLUP);   

  //Configure Timer2 to interrupt every 0.4ms for IR receiving: 
  TCCR2A = 0;  
  TCCR2B = 0;  
  TCNT2  = 0;  
  OCR2A  = 49;                                                                    // Interrupt every 0.4ms for IR receiving
  TCCR2A = 0b00000010;                                                            // Clear Timer on Compare match mode
  TCCR2B = 0b00000101;                                                            // ticks at 125KHz  
  TIMSK2 = 0b00000000;                                                            // Compare match on OCR2A interrupt (set this once signal is being received) TIMSK2 = 0b00000010;

  // Pin change interrupts:  
  attachInterrupt(digitalPinToInterrupt(3), ISR_LR, CHANGE);                      // Attach interrupt on pin INT1        (pin 3)   (FwdBckIn)
  PCMSK0 = 0b00001000;                                                            // Unmask interrupt on PCINT3          (pin 11)  (IR_In)
  PCMSK2 = 0b00000010;                                                            // Unmask interrupt on PCINT17         (pin 1)   (LfRghtIn)  
  PCICR = 0b00000101;                                                             // Enable Interrupts on PCI_2 and PCI_0

  // Detect whether an IR receiver or RF receiver is connected:
  IR_OR_PWM();                                                                    // Sets the RF_Receiver_Connected global variable to 1 if true otherwise 0 (BLOCKS until a signal is received)
  if(!RF_Receiver_Connected)
  {
    detachInterrupt(digitalPinToInterrupt(3));                                    // If No RF receiver attached disable the interrupts on these pins and use pin 3 for an ESC PWM signal
    PCMSK2 = 0b00000000; 
    pinMode(PWM_Pin, OUTPUT);
    PWM_Pulse_Width = 1000;
    
    Beep_Motors(4000, 100);                                                       // Beep twice if IR being used (secondary control...)
    delay(100);
  }

  Beep_Motors(3000, 100);    
}

//--- Main: ----------------------------------------------------------------------
void loop() {
  // If DMP initialisation failed just run without gyro control
  if (!dmpReady)  {                         // TRY WITHOUT GYRO
    while(1)  {
      if(!Low_Battery())  { 
        if(RF_Receiver_Connected) {                                             // If an RF receiver is connected take signals from this as it has better performance than an IR controller
          if(Process_PPM() == VALID)  {                                         // Process_PPM() returns 1 if valid PPM data is being received
          
            ENABLE_MOTORS;                                                      // Ensure outputs are enabled and go on to update them
            unsigned char Lout = constrain(128 + FwdBckPulseWdth_Safe + LfRghtPulseWdth_Safe, 0, 255);         
            unsigned char Rout = constrain(128 - FwdBckPulseWdth_Safe + LfRghtPulseWdth_Safe, 0, 255);         
            OCR1A = Lout;
            OCR1B = Rout;
          }              
        }
        else  {                                                                 // Otherwise we assume an IR receiver is connected. If it's not we shutdown the motors anyway
          if(Process_IR() == VALID) {                                           // Process_IR() returns 1 if valid IR data is being received 
            
            ENABLE_MOTORS;                                                      // Ensure outputs are enabled and go on to update them
            unsigned char Lout = constrain(128 + FwdBckPulseWdth_Safe + LfRghtPulseWdth_Safe, 0, 255);         
            unsigned char Rout = constrain(128 - FwdBckPulseWdth_Safe + LfRghtPulseWdth_Safe, 0, 255);         
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

  static char Been_Here_Before = 0;
  if(!Been_Here_Before) {                                                       // Get the gravity vector at start up. This does mean the robot must be switched on in it's UPRIGHT position
    mpu.dmpGetGravity(&Gravity_at_Startup, &q);
    Been_Here_Before = 1;
  } 
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//#######################################################################################################################################################################
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
char Process_PPM() 
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
  
  if(millis() - Time_at_Motor_Update < PPM_UPDATE_PERIOD) { 
    if(No_Signal_PWM > NO_SIGNAL_THESHOLD_PWM || Neutral_Input_Count_PWM >= 25)  {
      DISABLE_MOTORS;
      Yaw_setpoint = Yaw;
      return(0);
    }
    else  {
      return(1);  
    }    
  }  
  Time_at_Motor_Update += PPM_UPDATE_PERIOD;                                              

  //FwdBckPulseWdth_Safe = 0;                                                     // Uncommenting these -> If no data is until the no signal timeout, the motors will run using their last received data values,                                              
  //LfRghtPulseWdth_Safe = 0;                                                     // rather than stop. Can reduce jerkyness on poor signals but means robot doesn't stop on signal loss as quickly.

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
       

  if(Yaw_setpoint >= 360.0)                                                       // Keep Yaw_setpoint within 0-360 limits
      Yaw_setpoint -= 360.0;    
  else if(Yaw_setpoint < 0.0) 
      Yaw_setpoint += 360.0;          

  if(FwdBckPulse && LfRghtPulse)  {                                               // If we've just received 2 valid pulses, clear the No_Signal count    
      No_Signal_PWM = 0;  
    }  

  // 3) -----
  if(No_Signal_PWM > NO_SIGNAL_THESHOLD_PWM)  {     
    LED_OFF;                                                                      // Turn LED off if no signal is being received
    DISABLE_MOTORS;                                                               // Disable outputs  
           
    OCR1A = 128;
    OCR1B = 128; 
    return(0);
  }   

  // 4) -----
  if(abs(LfRghtPulseWdth_Safe) <= 10 && abs(FwdBckPulseWdth_Safe) <= 10) {        // If control sticks are in their neurtal position...
    Neutral_Input_Count_PWM++;
    
    if(Neutral_Input_Count_PWM >= 25) {
      Yaw_setpoint = Yaw;
      DISABLE_MOTORS;
      
      if(Neutral_Input_Count_PWM % 15 == 0) {                                     // Slow flash LED to indicate the PID algorithm is suspended due to inactivity
        digitalWrite(LED, !digitalRead(LED));      
      }     
       
      return(0);                                                                  // Return with a zero to disable motor activity
    }   
  }
  else  {
    Neutral_Input_Count_PWM = 0; 
  }
  
  LED_ON;                                                                         // Turn LED on to indicate signal being received  
  return(1);  
}

//--------------------------------------------------------------------------------
char Process_IR() 
{ 
// A few things about this routine:
//  1) It's run every IR_UPDATE_PERIOD ms
//  2) It checks if any IR Data is being recieved. If it is it will always be returning a 1 otherwise a 0 
//  3) Turns LED on to indiciate circuit is receiving valid IR data from receiver  
//  4) If the joysticks go still for a while, idle.   

  // 1) ----- 
  static int Neutral_Input_Count_IR = 25;
  static unsigned long Time_at_Motor_Update; 
  static int No_Signal_IR = NO_SIGNAL_THESHOLD_IR;                                // Assume no signal at startup
  
  if(millis() - Time_at_Motor_Update < IR_UPDATE_PERIOD) { 
    if(No_Signal_IR > NO_SIGNAL_THESHOLD_IR || Neutral_Input_Count_IR >= 25)  {
      DISABLE_MOTORS;                                                      
      Yaw_setpoint = Yaw;
      return(0);
    }
    else  {
      return(1);  
    }    
  }  
  Time_at_Motor_Update += IR_UPDATE_PERIOD;                                      
  

  //FwdBckPulseWdth_Safe = 0;                                                     // Uncommenting these -> If no data is until the no signal timeout, the motors will run using their last received data values,                                              
  //LfRghtPulseWdth_Safe = 0;                                                     // rather than stop. Can reduce jerkyness on poor signals but means robot doesn't stop on signal loss as quickly.
    
  if(Decode_Flag)  {                                                              // If IR Data has been received
    unsigned long IR_Data = Decode();                                             // Decode the data
    if(IR_Data)   {                                                               // If The data contains valid information clear the no signal count and extract the info
      No_Signal_IR = 0;
      
      Yaw_setpoint = (float)map(((IR_Data >> 12) & 0xFF), 0, 255, 0, 359);
      FwdBckPulseWdth_Safe = ((IR_Data >> 4) & 0xFF) - 128;
      FwdBckPulseWdth_Safe = (FwdBckPulseWdth_Safe * 3) / 4; 
       
      static float Old_Yaw_setpoint;
      LfRghtPulseWdth_Safe = round((Yaw_setpoint - Old_Yaw_setpoint) * 5.0);      // Incase the Gyro chip isn't populated this still allows for IR robot control
      Old_Yaw_setpoint = Yaw_setpoint;

      // The last 4 bits of the packet contain info on whether to spin the ESC, or change the PID parameters etc the 4th bit will be for ESC control        
      Update_PID_Values(IR_Data & 0x7);
    }
    else  
      No_Signal_IR++;                                                             // If the data packet was empty which occurs if there was an error etc, increase no signal count          
  }
  else  
    No_Signal_IR++;                                                               // Or if there was no data received at all, increase no signal count

  // 3) -----
  if(No_Signal_IR > NO_SIGNAL_THESHOLD_IR)  {     
    LED_OFF;                                                                      // Turn LED off if no signal is being received
    DISABLE_MOTORS;                                                               // Disable outputs  
    Access_EEPROM(1);                                                             // Write PID parameters to EEPROM if they have been updated.

    OCR1A = 128;
    OCR1B = 128; 

    return(0);
  }

  // 4) -----
  if(abs(LfRghtPulseWdth_Safe) <= 10 && abs(FwdBckPulseWdth_Safe) <= 10) {        // If control sticks are in their neurtal position...
    Neutral_Input_Count_IR++;
    
    if(Neutral_Input_Count_IR >= 25) {
      Yaw_setpoint = Yaw;
      DISABLE_MOTORS;
      
      if(Neutral_Input_Count_IR % 15 == 0) {                                      // Slow flash LED to indicate the PID algorithm is suspended due to inactivity
        digitalWrite(LED, !digitalRead(LED));      
      }     
             
      return(0);                                                                  // Return with a zero to disable motor activity
    }   
  }
  else  {
    Neutral_Input_Count_IR = 0; 
  }  
  
  LED_ON;                                                                         // Turn LED on to indicate signal being received ok  
  return(1);  
}

//--------------------------------------------------------------------------------
long Decode()
{
// Encoded_Data from ISR is processed to a long. If data is erronous returns 0
                                                                            
  char i, index; 
  char BitH;
  char BitL;
  unsigned long IR_Data = 0;
  unsigned long Encoded_Data_Safe[2];
  
  Encoded_Data_Safe[0] = Encoded_Data[0];
  Encoded_Data_Safe[1] = Encoded_Data[1];
  Decode_Flag = 0;

  index = (EXPECTED_BITS - 1);
  
  if(EXPECTED_BITS > 16)  {    
    for(i = (((EXPECTED_BITS - 16) * 2) - 1); i >= 0;i -= 2)
    {
      BitH = bitRead(Encoded_Data_Safe[1], i);
      BitL = bitRead(Encoded_Data_Safe[1], i-1);
  
      if(BitH && !BitL)
        bitSet(IR_Data, index); 
      else if(!BitH && BitL)
        bitClear(IR_Data, index); 
      else
        return(0);                                                                // Erronous data, ignore packet
  
      index--;
    } 
    
    for(i = 31; i >= 0;i -= 2)
    {
      BitH = bitRead(Encoded_Data_Safe[0], i);
      BitL = bitRead(Encoded_Data_Safe[0], i-1);
  
      if(BitH && !BitL)
        bitSet(IR_Data, index); 
      else if(!BitH && BitL)
        bitClear(IR_Data, index); 
      else
        return(0);                                                                // Erronous data, ignore packet
  
      index--;
    }       
  }
  else  {
    for(i = ((EXPECTED_BITS * 2) - 1); i >= 0;i -= 2)
    {
      BitH = bitRead(Encoded_Data_Safe[0], i);
      BitL = bitRead(Encoded_Data_Safe[0], i-1);
  
      if(BitH && !BitL)
        bitSet(IR_Data, index); 
      else if(!BitH && BitL)
        bitClear(IR_Data, index); 
      else
        return(0);                                                                // Erronous data, ignore packet
  
      index--;
    }     
  }
  
  return(IR_Data);
}

//--------------------------------------------------------------------------------
char PID_Routine()
{
// So we need LfRghtPulseWdth to adjust itself according to how much error there is. Here's a little PID controller to do that. 
// It uses the global variable, Yaw_Setpoint & needs FwdBckPulseWdth_Safe for speed control
// We deduce whether the device has been flipped by comparing the current gravity vector against the startup gravity vector. If their dot product
// is -ve we've been flipped. In this case we need to ?? 1) Error = -Error 2) LfRghtPulseWdth_Safe = -LfRghtPulseWdth_Safe etc....

  static unsigned long Time_at_Motor_Update;
  long FwdBckPulseWdth_Safe_Temp = FwdBckPulseWdth_Safe;
  unsigned long delta_t;

  delta_t = micros() - Time_at_Motor_Update;  
  if(delta_t < 10000)                                                             // Run this routine every 10ms
    return(0); 
  Time_at_Motor_Update += 10000;                                                                                            

  float Error, Output, dInput;
  static float ErrorSum, LastYaw; 

  if(Upside_Down()) {
    float Inverted_Yaw_setpoint = 180.0 - Yaw_setpoint;
    
    if(Inverted_Yaw_setpoint >= 360.0)                                            // Keep Inverted_Yaw_setpoint within 0-360 limits
        Inverted_Yaw_setpoint -= 360.0;    
    else if(Inverted_Yaw_setpoint < 0.0) 
        Inverted_Yaw_setpoint += 360.0;    
          
    FwdBckPulseWdth_Safe_Temp = -FwdBckPulseWdth_Safe_Temp;   
    Error = -Turn_Error(Inverted_Yaw_setpoint, Yaw);
    dInput = (LastYaw - Yaw) / delta_t;
  }
  else  {
    Error = Turn_Error(Yaw_setpoint, Yaw); 
    dInput = (Yaw - LastYaw) / delta_t;
  }  
  
  ErrorSum += (Error * Ki) * delta_t;                                             // delta_t ~= 10_000, Also "bumpless"                                                     
  ErrorSum = constrain(ErrorSum, -100.0, 100.0);
  
  LastYaw = Yaw;
  
  Output = (Error * Kp) + ErrorSum - (dInput * Kd);
  
  LfRghtPulseWdth_Safe = round(Output);
  LfRghtPulseWdth_Safe = constrain(LfRghtPulseWdth_Safe, -100, 100);

  ENABLE_MOTORS;                                                                  // Ensure outputs are enabled 
  static unsigned char BufferA[ROLLING_AVG_FILTER_LENGTH], IndexA;
  static unsigned char BufferB[ROLLING_AVG_FILTER_LENGTH], IndexB; 
  unsigned char  Lout = Rolling_Avg(BufferA, constrain(128 + FwdBckPulseWdth_Safe_Temp + LfRghtPulseWdth_Safe, 0, 255));       //Rolling_avg not actually working yet!  
  unsigned char Rout = Rolling_Avg(BufferB, constrain(128 - FwdBckPulseWdth_Safe_Temp + LfRghtPulseWdth_Safe, 0, 255));         
  OCR1A = Lout;
  OCR1B = Rout;

  // Blocking code to generate a PWM pulse should PWM_Pulse_Width be in 800-2000 range.
  if(PWM_Pulse_Width > 800) {
    if(PWM_Pulse_Width < 2000) {
      static char Toggle = 0;
      if(Toggle--)  
        PWM_PulseOut(PWM_Pulse_Width);                                            // We only send the pulse every other call to this function ie. every ~20ms
      else
        Toggle++;
    }      
  }   

  return(1);
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
      Kp = max(Kp, 0.0);                                                          // constrain the min value to 0
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
// If Read_Write = 1 this routine writes the current PID parameters to EEPROM otherwise it reads them

  if(Read_Write)
  {
    EEPROM.put(0, Kp);
    EEPROM.put(4, Ki);
    EEPROM.put(8, Kd);
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

//--------------------------------------------------------------------------------
void IR_OR_PWM()  
{
  delay(200);
  
  while(1)                                                                        // Keep looping until either an IR or RF signal is being received since there's no point continuing otherwise!
  {    
    LfRghtPulseWdth = 0;
    FwdBckPulseWdth = 0;
    Decode_Flag = 0;

    LED_OFF;
    delay(125);
    LED_ON;
    delay(125);
  
    if(LfRghtPulseWdth && FwdBckPulseWdth)  {                                     // If we're receiving RF data return with RF_Receiver_Connected = 1
      RF_Receiver_Connected = 1;
      return;
    }
      
    if(Decode_Flag) {
      if(Decode())  {                                                             // If we're receiving a proper manchester encoded signal from the IR return with RF_Receiver_Connected = 0;                         
        RF_Receiver_Connected = 0;
        return;      
      }
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
    LfRghtPulseWdth = micros() - Time_at_Rise_FB;
} 

//--------------------------------------------------------------------------------
void ISR_LR()                                                                     // Change in FwdBckIn Pin of PWM in
{
  static unsigned long Time_at_Rise_LR = 0;
  
  if (FWDBCKIN_STATE == HIGH)
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
  
  if(bit_index)                                                                   // If a receive has started IF TIMER2 ONLY GETS STARTED AFTER A START PULSE IS THIS NEEDED?
  {      
    bit_index--;
    
    Time_Diff = micros() - TimeA;
    TimeA = Time_Diff + TimeA;                                                    // This is faster than TimeA = micros() and more accurate

    if(State == HIGH) {
      Total_Time_High += Time_Diff;  
    }

    if(bit_index == (EXPECTED_BITS * 2))                                          // If this is the first bit... 
    {
      if(Total_Time_High > 475 && Total_Time_High < 625)  {                       // Channel 1 start bits are ~600us long. COULD NARROW THESE WONDOWS DOWN SOMEWHAT.
        Total_Time_High = 0;
        OCR2A = 49;                                                               // Interrupt every 400us from now until the end of the packet.
        return;
      }
      else if (Total_Time_High > 175 && Total_Time_High < 325)  {                 // Channel 2 start bits are ~250us long. COULD NARROW THESE WONDOWS DOWN SOMEWHAT.
        Total_Time_High = 0;
        OCR2A = 49;                                                               // Interrupt every 400us from now until the end of the packet.
        return;
      }
      else  {
        bit_index = 0;                                                            // Cancel this receive and search for start bit again.
        Total_Time_High = 0;
        TIMSK2 = 0b00000000;                                                      // Disable OCR2A interrupts      
        TimeA -= 19000;                                                           // Wait for only 1ms before searching for starts again.
        return;
      }
    }    

    if(bit_index > 31)  {
      if(Total_Time_High > 200) {
        bitWrite(DataH, (bit_index - 32), 1);           
      }
      else  {
        bitWrite(DataH, (bit_index - 32), 0); 
      }        
    }
    else  {
      if(Total_Time_High > 200) {
        bitWrite(DataL, bit_index, 1);           
      }
      else  {
        bitWrite(DataL, bit_index, 0); 
      }  
    } 
    
    Total_Time_High = 0;

    if(!bit_index)  {
      TIMSK2 = 0b00000000;                                                        // Disable OCR2A interrupts
      
      Decode_Flag = 1;
      Encoded_Data[0] = DataL; 
      Encoded_Data[1] = DataH;        
    }
  }
} 

//--------------------------------------------------------------------------------
ISR (PCINT0_vect)                                                                 // IR_Pin Change
{ 
  Time_Diff = micros() - TimeA;                                                  
  State = IR_IN_STATE;

  if(bit_index)                                                                   // If signal has started
  {    
    TimeA = Time_Diff + TimeA;                                                    // TimeA = micros() esentially, the time at beginning of this interrupt
    
    if(State == LOW)                                                              //Signal must have just gone low so Time_Diff holds the most recent high pulse width
    {
      Total_Time_High += Time_Diff;
      return;
    }    
  } 

  // Here we detect whether the the signal has just started. If it has, set bit_index to EXPECTED_BITS*2 + 1. We only detect start bits 20ms after the last packet finishes (i.e just befor next pulse 
  // is due to arrive. This way we should sync with the transmitter and improve our robustness to interference etc.
  if (!State && (Time_Diff > 20000))
  {   
    TimeA = Time_Diff + TimeA;                                                    // TimeA = micros() esentially, the time at beginning of this interrupt
    
    TCNT2 = 0;                                                                    // Reset the counter.
    OCR2A = 99;                                                                   // Interrupt in 800us - Start bit measurement
    
    TIFR2 = 0b00000010;                                                           // Reset any counter interrupt 
    TIMSK2 = 0b00000010;                                                          // Enable interrupts on OCR2A compare match          
    bit_index = (EXPECTED_BITS * 2) + 1;                                          

    return;        
  }
}
