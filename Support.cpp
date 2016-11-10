#include "Support.h"
#include "Arduino.h"
#include <avr/power.h>
#include <avr/sleep.h>
#include <math.h>
#include <EEPROM.h>

float Turn_Error(float Yaw_setpoint, float Yaw)
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
// If there has been low power for 30 seconds we enter a non-recoverable low power sleep mode.
// Takes a bit of an average of the ADC readings to prevent spurious low readings causing a glitch
  
  static unsigned long Time_at_LED_Change = 0;  
  static long Time_When_Last_Had_Power = millis();
  static bool Low_Battery_Flag = 0;
  static byte index = 0;
  static int BVoltage_Buffer[BATTERY_AVG_FILTER_LENGTH];
  long BVoltage_Avg = 0;   
  
  BVoltage_Buffer[index++] = analogRead(Vsense) * VOLTAGE_SENSE_CONSTANT;         // Convert to millivolts and add to buffer

  // This section only runs the first time this routine is called. It detects how many cells the battery is made of
  static char Number_Of_Cells = 0;
  if(!Number_Of_Cells) {
    if(BVoltage_Buffer[0] >= 5500)
      Number_Of_Cells = 2;
    else
      Number_Of_Cells = 1;
  }
  
  // Perform the averaging:
  if (Low_Battery_Flag) {                                                         // If the low battery flag has been set - calculate average every call to this function and check...
    if (index >= BATTERY_AVG_FILTER_LENGTH) {
      index = 0;
    }

    BVoltage_Avg = 0;
    for(byte i = 0;i < BATTERY_AVG_FILTER_LENGTH;i++)  {
       BVoltage_Avg += BVoltage_Buffer[i];
    }
    
    BVoltage_Avg /= BATTERY_AVG_FILTER_LENGTH;    
  }
  
  else if (index >= BATTERY_AVG_FILTER_LENGTH)  {                                 // If low battery flag hasn't been set, only check thresholds every BATTERY_AVG_FILTER_LENGTH calls to this function
    index = 0;

    BVoltage_Avg = 0;
    for(byte i = 0;i < BATTERY_AVG_FILTER_LENGTH;i++)  {
       BVoltage_Avg += BVoltage_Buffer[i];
    }
    BVoltage_Avg /= BATTERY_AVG_FILTER_LENGTH;
  }
  
  else  {
    return(0);
  }
  
  // Check Avg Battery voltage against defined setpoints:
  if(Number_Of_Cells == 2)  {
    if(BVoltage_Avg > BATTERY_THRESHOLD_LOW_2S && !Low_Battery_Flag)
      {
        Time_When_Last_Had_Power = millis();                                    
        return(0);
      }
    if(BVoltage_Avg > BATTERY_THRESHOLD_HGH_2S)
      {
        Time_When_Last_Had_Power = millis();
        Low_Battery_Flag = 0;
        return(0);
      }
  }
  else  {
    if(BVoltage_Avg > BATTERY_THRESHOLD_LOW_1S && !Low_Battery_Flag)
      {
        Time_When_Last_Had_Power = millis();
        return(0);
      }
    if(BVoltage_Avg > BATTERY_THRESHOLD_HGH_1S)
      {
        Time_When_Last_Had_Power = millis();
        Low_Battery_Flag = 0;
        return(0);
      } 
  }   

  Low_Battery_Flag = 1;  
  DISABLE_MOTORS;                                                                 // Disable all outputs

  if(millis() - Time_at_LED_Change > 75)
  {
    digitalWrite(LED, !digitalRead(LED));                                         // Fast flash LED
    Time_at_LED_Change = millis();
  }  

  if(millis() - Time_When_Last_Had_Power > 30000)                                 // If we last had power over 30 seconds ago - enter super low power mode
  {
    System_Power_Down();
  }
  
  return(1);  
}

//--------------------------------------------------------------------------------
void System_Power_Down()
{
  LED_OFF;
  DISABLE_MOTORS;
  Sleep_6050();                                                                   // Sleep the IMU

  detachInterrupt(digitalPinToInterrupt(3));                                      // Stop all interrupts
  detachInterrupt(digitalPinToInterrupt(2));
  PCICR = 0b00000000;

  TCCR0A = 0; TCCR0B = 0;                                                         // Stop all the timers
  TCCR1A = 0; TCCR1B = 0;
  TCCR2A = 0; TCCR2B = 0;
  
  pinMode(LED, INPUT);                                                            // Set pins to outputs
  pinMode(MotorL, INPUT);  
  pinMode(MotorR, INPUT);
  pinMode(IR_Pos, INPUT);                          
  pinMode(IR_GND, INPUT);
  pinMode(IR_In, INPUT);
  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);                                            // The only way to get out of this will be a device reset!
  sleep_enable();                                                                 // Set sleep enable (SE) bit:
  sleep_mode();                                                                   // Put the device to sleep:

  while(1) {}
}

//--------------------------------------------------------------------------------
char * float2s(float f, unsigned int digits)
{
  int index = 0;
  static char s[16];                                                              // buffer to build string representation
 
  if (f < 0.0) {                                                                  // handle sign
   s[index++] = '-';
   f = -f;
  } 
  
  if (isinf(f))  {                                                                // handle infinite values
   strcpy(&s[index], "INF");
   return s;
  }
  
  if (isnan(f))  {                                                                // handle Not a Number
   strcpy(&s[index], "NaN");
   return s;
  }
  
  if (digits > 6)                                                                 // max digits
    digits = 6;
  long multiplier = pow(10, digits);                                              // fix int => long
  
  int exponent = int(log10(f));
  float g = f / pow(10, exponent);
  if ((g < 1.0) && (g != 0.0))  {
   g *= 10;
   exponent--;
  }
  
  long whole = long(g);                                                           // single digit
  long part = long((g-whole)*multiplier);                                         // # digits
  char format[16];
  
  sprintf(format, "%%ld.%%0%dld E%%+d", digits);
  sprintf(&s[index], format, whole, part, exponent);
  
  return s;
}

//--------------------------------------------------------------------------------
void Beep_Motors(unsigned long Frequency, unsigned long Duration)
{  
    ENABLE_MOTORS;

    unsigned long Time_Period = 1000000 / Frequency;
    unsigned long Start_Time = millis();

    while(millis() - Start_Time < Duration)
    {
      OCR1A = 128 + MOTOR_BEEP_AMPLITUDE;
      OCR1B = 128 + MOTOR_BEEP_AMPLITUDE; 
      delayMicroseconds(Time_Period/2);   
         
      OCR1A = 128 - MOTOR_BEEP_AMPLITUDE;
      OCR1B = 128 - MOTOR_BEEP_AMPLITUDE; 
      delayMicroseconds(Time_Period/2);      
    }

    OCR1A = 128;
    OCR1B = 128;
    DISABLE_MOTORS;  
}

//--------------------------------------------------------------------------------
char Get_Channel_From_EEPROM(char Read_Write, char New_Channel)
{
// If Read_Write = 1 this routine writes New_Channel value to EEPROM, if 0 it returns the current channel number
  char return_val;
  New_Channel = constrain(New_Channel, 0, 1);
  
  if(Read_Write)
  {
    EEPROM.put(12, New_Channel);
    return_val = New_Channel;
  }
  else
  {
    EEPROM.get(12, return_val);   
  }
  
  return(return_val);  
}


//--------------------------------------------------------------------------------
float Calculate_Joy_Stick_Angle(int X, int Y)
{  
  static float JoyStick_Angle;  

  JoyStick_Angle = atan2(X, Y) * (180/PI);                                      // Calculate new value

  return(JoyStick_Angle);
}

//--------------------------------------------------------------------------------
long Calculate_Joy_Stick_Magnitude(int X, int Y)
{  
  long JoyStick_Sq_Magnitude;

  JoyStick_Sq_Magnitude = (X*X) + (Y*Y);

  return(JoyStick_Sq_Magnitude);
}



















