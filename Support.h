#ifndef _SUPPORT_H_
#define _SUPPORT_H_


//--- Defines: -------------------------------------------------------------------
#define   EXPECTED_BITS               20                                          // For IR receiving. The number of bits we expect to receive

#define   NO_SIGNAL_THESHOLD_PWM      10                                          // Number of erronous PPM signals received before we deduce no signal is being received
#define   NO_SIGNAL_THESHOLD_IR       10                                          // Number of erronous PPM signals received before we deduce no signal is being received

#define   VALID                       1
#define   INVALID                     0

#define   IR_PACKET_SEARCH_ENABLED    PCMSK0
#define   ENABLE_IR_PACKET_SEARCH     0b00001000
#define   DISABLE_IR_PACKET_SEARCH    0b00000000  
#define   CLEAR_PCI0_INTERRUPT        0b00000001
#define   CLEAR_OC2A_INTERRUPT        0b00000010
#define   CLEAR_OC2B_INTERRUPT        0b00000100

#define   OCR2B_CMP_INTS_ENABLED      0b00000100 
          
#define   ENABLE_MOTORS               digitalWrite(MEnble, HIGH)
#define   DISABLE_MOTORS              digitalWrite(MEnble, LOW)
#define   LED_ON                      digitalWrite(LED, HIGH)
#define   LED_OFF                     digitalWrite(LED, LOW)
#define   PWM_PIN_HIGH                digitalWrite(PWM_Pin, HIGH)
#define   PWM_PIN_LOW                 digitalWrite(PWM_Pin, LOW)

#define   FWDBCKIN_STATE              digitalRead(FwdBckIn)
#define   LFRGHTIN_STATE              digitalRead(LfRghtIn)
#define   IR_IN_STATE                 digitalRead(IR_In)

#define   MOTOR_BEEP_AMPLITUDE        100                                         // 0 - 125 full range 
#define   ROLLING_AVG_FILTER_LENGTH   5
#define   IR_UPDATE_PERIOD            50
#define   PPM_UPDATE_PERIOD           38


//--- Low_Battery() related constants:
#define   VOLTAGE_SENSE_CONSTANT      10.08                                       // 24.71; - This is the value for the Green LED GMC that I built 1st
#define   BATTERY_THRESHOLD_LOW_2S    6300                                        // If voltage falls below this, enter sleep
#define   BATTERY_THRESHOLD_HGH_2S    7000                                        // If voltage rises above this, turn active again
#define   BATTERY_THRESHOLD_LOW_1S    3150                                        // If voltage falls below this, enter sleep
#define   BATTERY_THRESHOLD_HGH_1S    3500                                        // If voltage rises above this, turn active again


//--- Pin definitions: -----------------------------------------------------------
#define   PWM_Pin           3                                                     // Output PWM, used if IR detected and hence FwdBckIn not being used etc

#define   FwdBckIn          3                                                     // This is INT1
#define   LfRghtIn          1                                                     // This is PCINT17 (calls PCINT2_vect). Although this is also the Tx pin

#define   MEnble            8                                                     // Enable Motors
#define   MotorL            9                                                     // Left Motors
#define   MotorR            10                                                    // Right Motors

#define   IR_Pos            13                                                    // Convienient use of pins to supply power to IR reciever.
#define   IR_GND            12
#define   IR_In             11                                                    // PB3 which is PCINT3

#define   LED               0                                                     // LED Pin. Although this is also the Rx pin
#define   Vsense            A2                                                    // Voltage sensing (150ohm to 1K divider)


//--- Routines -------------------------------------------------------------------
float Turn_Error(float, float);                                                   // From the Yaw & Yaw_setpoint, this routine returns how many degrees we are from where we want to be. 
int Low_Battery(void);                                                            // Returns 0 if battery is sufficiently charged, 1 otherwise. Includes hysterisis & LED flashing for low battery (Non-blocking)
char * float2s(float f, unsigned int digits);                                     // Converts a float to a string in standard form
void Beep_Motors(unsigned long Frequency, unsigned long Duration);                // Uses the motors as speakers to produce noise
void PWM_PulseOut(int us);                                                        // A blocking function that outputs a pulse of duration us. 
unsigned char Rolling_Avg(unsigned char *Buffer, unsigned char Value);            // UNFINISHED!


#endif 






