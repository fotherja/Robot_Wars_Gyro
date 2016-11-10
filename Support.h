#ifndef _SUPPORT_H_
#define _SUPPORT_H_


//--- Defines: -------------------------------------------------------------------
// User adjustable:
#define   ROLLING_AVG_FILTER_LENGTH   2                                           // Rolling average filter length. Smooths acceleration of motors.
#define   BATTERY_AVG_FILTER_LENGTH   5                                           // Smooths the ADC readings of the battery voltage to avoid spurious drops from causing shutdown     

#define   NO_SIGNAL_THESHOLD_PWM      10                                          // Number of erronous PPM signals received before we deduce no signal is being received
#define   NO_SIGNAL_THESHOLD_IR       10                                          // Number of erronous PPM signals received before we deduce no signal is being received
#define   NEUTRAL_THRESHOLD_COUNT     20

#define   MOTOR_BEEP_AMPLITUDE        100                                         // 0 - 125 full range 
#define   IR_UPDATE_PERIOD            40
#define   PPM_UPDATE_PERIOD           40

#define   DUTY_MAX                    255
#define   DUTY_MIN                    0

//--- Low_Battery() related constants:
#define   VOLTAGE_SENSE_CONSTANT      10                                          // 24.71 - This is the value for the Green LED GMC that I built 1st otherwise 10.08 - ROUND TO NEAREST INT
#define   BATTERY_THRESHOLD_LOW_2S    6300                                        // If voltage falls below this, enter sleep
#define   BATTERY_THRESHOLD_HGH_2S    7000                                        // If voltage rises above this, turn active again
#define   BATTERY_THRESHOLD_LOW_1S    3150                                        // If voltage falls below this, enter sleep
#define   BATTERY_THRESHOLD_HGH_1S    3500                                        // If voltage rises above this, turn active again

// Non adjustable:
#define   VALID                       1
#define   INVALID                     0
#define   EEPROM_READ                 0
#define   EEPROM_WRITE                1

#define   IR_CTRL                     0
#define   RF_2CH_CTRL                 1
#define   RF_3CH_CTRL                 2
#define   RF_PPM                      3

#define   IR_PACKET_SEARCH_ENABLED    PCMSK0
#define   ENABLE_IR_PACKET_SEARCH     0b00001000
#define   DISABLE_IR_PACKET_SEARCH    0b00000000  
#define   CLEAR_PCI0_INTERRUPT        0b00000001
#define   CLEAR_OC2A_INTERRUPT        0b00000010
#define   CLEAR_OC2B_INTERRUPT        0b00000100

#define   OCR2B_CMP_INTS_ENABLED      0b00000100 
          
#define   ENABLE_MOTORS               PORTB |= 0x01                               // digitalWrite(MEnble, HIGH)   
#define   DISABLE_MOTORS              PORTB &= 0xFE                               // digitalWrite(MEnble, LOW)    
#define   LED_ON                      PORTD |= 0x01                               // digitalWrite(LED, HIGH)       
#define   LED_OFF                     PORTD &= 0xFE                               // digitalWrite(LED, LOW)       
#define   PWM_PIN_HIGH                PORTD |= 0x08                               // digitalWrite(PWM_Pin, HIGH)  
#define   PWM_PIN_LOW                 PORTD &= 0xF7                               // digitalWrite(PWM_Pin, LOW)   

#define   MOTORS_ENABLED              PINB & 0x01                                 //digitalRead(MEnble)           
#define   FWDBCKIN_STATE              PIND & 0x08                                 //digitalRead(FwdBckIn)         
#define   LFRGHTIN_STATE              PIND & 0x02                                 //digitalRead(LfRghtIn)         
#define   AUXIN_STATE                 PINB & 0x08                                 //digitalRead(AuxPWMIn)         
#define   IR_IN_STATE                 PINB & 0x08                                 //digitalRead(IR_In) 
#define   PWM_PIN_IS_HIGH             PIND & 0x08                                 //digitalRead(PWM_Pin)

//--- Pin definitions: -----------------------------------------------------------
#define   PWM_Pin                     3                                           // Output PWM, used if IR detected and hence FwdBckIn not being used etc

#define   FwdBckIn                    3                                           // This is INT1
#define   LfRghtIn                    1                                           // This is PCINT17 (calls PCINT2_vect). Although this is also the Tx pin
#define   AuxPWMIn                    11                                          // PB3 which is PCINT3

#define   MEnble                      8                                           // Enable Motors
#define   MotorL                      9                                           // Left Motors
#define   MotorR                      10                                          // Right Motors

#define   IR_Pos                      13                                          // Convienient use of pins to supply power to IR reciever.
#define   IR_GND                      12
#define   IR_In                       11                                          // PB3 which is PCINT3

#define   LED                         0                                           // LED Pin. Although this is also the Rx pin
#define   Vsense                      A2                                          // Voltage sensing (150ohm to 1K divider)

//--- Routines -------------------------------------------------------------------
float Turn_Error(float, float);                                                   // From the Yaw & Yaw_setpoint, this routine returns how many degrees we are from where we want to be. 
float Calculate_Joy_Stick_Angle(int X, int Y);                                    // Given x and y values this returns the angle of the joystick
long Calculate_Joy_Stick_Magnitude(int X, int Y);                                // Given x and y values this returns a square magnitude of the joystick deviation
int Low_Battery(void);                                                            // Returns 0 if battery is sufficiently charged, 1 otherwise. Includes hysterisis & LED flashing for low battery (Non-blocking)
char * float2s(float f, unsigned int digits);                                     // Converts a float to a string in standard form
void Beep_Motors(unsigned long Frequency, unsigned long Duration);                // Uses the motors as speakers to produce noise
char Get_Channel_From_EEPROM(char Read_Write, char New_Channel);                  // Writes or reads the current IR channel from/to EEPROM.
void System_Power_Down(void);                                                     // Completely shuts down the GMC: Gyro, Motor drivers, IR receiver, LED, the arduino clock, all interrupts etc
void Sleep_6050(void);                                                            // ALTHOUGH this function is in the main file "Robot_Wars_Gyro" or whatever, this needs referencing here

//--- Variables ------------------------------------------------------------------


//--- Classes --------------------------------------------------------------------


#endif 






