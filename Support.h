#ifndef _SUPPORT_H_
#define _SUPPORT_H_


//--- Defines: -------------------------------------------------------------------
#define   EXPECTED_BITS               20                                          // For IR receiving. The number of bits we expect to receive

#define   NO_SIGNAL_THESHOLD_PWM      10                                          // Number of erronous PPM signals received before we deduce no signal is being received
#define   NO_SIGNAL_THESHOLD_IR       10                                          // Number of erronous PPM signals received before we deduce no signal is being received

#define   VALID                       1
#define   INVALID                     0

#define   ENABLE_MOTORS               digitalWrite(MEnble, HIGH)
#define   DISABLE_MOTORS              digitalWrite(MEnble, LOW)
#define   LED_ON                      digitalWrite(LED, HIGH)
#define   LED_OFF                     digitalWrite(LED, LOW)

#define   MOTOR_BEEP_AMPLITUDE        100                                         // 0 - 125 full range 

//--- Low_Battery() related constants:
#define   VOLTAGE_SENSE_CONSTANT      10.08                                       // 24.71; - This is the value for the Green LED GMC that I built 1st
#define   BATTERY_THRESHOLD_LOW_2S    6300                                        // If voltage falls below this, enter sleep
#define   BATTERY_THRESHOLD_HGH_2S    7000                                        // If voltage rises above this, turn active again
#define   BATTERY_THRESHOLD_LOW_1S    3150                                        // If voltage falls below this, enter sleep
#define   BATTERY_THRESHOLD_HGH_1S    3500                                        // If voltage rises above this, turn active again


//--- Pin definitions: -----------------------------------------------------------
#define   ESC_PWM_Pin       3                                                     // Output PWM, used if IR detected and hence FwdBckIn not being used etc

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
char * float2s(float f, unsigned int digits);
void Beep_Motors(unsigned long Frequency, unsigned long Duration);                // Uses the motors as speakers to produce noise



#endif 






