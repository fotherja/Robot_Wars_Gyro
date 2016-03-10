#include "Support.h"
#include "Arduino.h"
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
  
  static unsigned long Time_at_LED_Change = 0;  
  static bool Low_Battery_Flag = 0;
  
  float BVoltage = analogRead(Vsense) * VOLTAGE_SENSE_CONSTANT;                   //Convert to millivolts

  // This section only runs the first time this routine is called. It detects how many cells the battery is made of
  static char Number_Of_Cells = 0;
  if(!Number_Of_Cells) {
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
  DISABLE_MOTORS;                                                                 //Disable all outputs

  if(millis() - Time_at_LED_Change > 100)
  {
    digitalWrite(LED, !digitalRead(LED));                                         // Flash LED
    Time_at_LED_Change = millis();
  }  
  
  return(1);  
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
void PWM_PulseOut(int us)
{
   PWM_PIN_HIGH;
   delayMicroseconds(us);
   PWM_PIN_LOW;
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








