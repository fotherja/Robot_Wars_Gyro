# Project Overview
This is the Arduino code for the Gyro Motor Controller (GMC): A gyro stabilised mixer for small DC powered robots. The GMC allows for signals to be received either by conventional RF by connecting it to an RF receiver, or via Infra-Red by attachment of a IR receiver module that directly mounts on the board. 

<p align="center">
  <img src="http://www.jamesfotherby.com/Images/Robot_Wars/Whole_Circuit.JPG" width="60%">
</p>

# Features:
 - Automatic detection of RF or IR connectivity
 - Automatic detection of 2 channel, 3 channel, or [PPM] in the case of RF control
 - 1S or 2S LiPo low battery detection and ultra-low power shutdown
 - PID algorithm to reach the Yaw_setpoint as fast as possible - can be tuned on the fly via IR
 - A PCM output channel to control a servo or ESC when using IR
 - 2 IR channels supported on the same IR frequency
 - 5v output to supply an RF receiver
 - 2 x 3A Motor channels

# Further Discussion:
If only 2 RF channels are connected one channel is used for speed control and the other is used to set the turn rate. If 3 channels are connected Joystick style turning is possible (whereby the robot points in the same direction as a 2 axis joystick).

# Discussion of IR control:
The IR protocols that have been developed for the GMC are designed to be as robust to interference as possible. The manchester encoded signal that is expected provides very robust error detection. IR has been found to work well and allows you to do away with a bulky RF receiver.

The IR transmitter code is available at https://github.com/fotherja/Robot_Wars_IR_Tx. It allows control via a Nintendo game gube or [by plugging into the trainer port of a standard RF transmitter.]

# LED Indications:
 - Very Fast Flashing - PID parameters being altered
 - Fast flashing      - Low battery - Ultra-low power shutdown occurs if this state remains for 30+ seconds
 - Medium Fast        - Searching for RF or IR signal on start up
 - Slow flashing      - Idling because the user isn't asking for any movement
 - LED Off            - No signal being received

Further to this, the attached motors will beep once if RF control has been detected and twice for IR

# Code Discussion:
The code uses a PID control algorithm running at 100Hz to to keep the robot pointing in the direction stored in the global variable, Yaw_Setpoint. The remote control (IR or RF) changes this variable and the robot attempts to follow. The other input is the speed value, which just moves the robot back and forth.

The measured Yaw is also updated at 100Hz by the DMP on the 6050 IMU together with some additional processing on the Arduino. The PID algorithm works to minimsie the Error = Yaw_setpoint - Yaw. In reality it's a bit more complicated than this since the motors must act to turn the robot in whichever direction will turn the robot to Yaw_setpoint fastest, and we've got to adjust things slightly if the robot is upside down.

62.5KHz PWM signals feed each of the left and right motors. The duty cycles to each motor controls their rotational speeds and this is what's adjusted by the PID algorithm.

If RF mode is used, interrupts on each of the connected channels measure the pulse high times. These times are then used to set speed and Yaw_setpoint in the case of 3 ch control, or speed and rate of turn in the case of 2 ch control.

If IR mode is used, when the start of a packet is detected a timer is started which interrupts every sub-bit in the manchester encoded signal. If the average high time > average low time for the previous sub-bit then we deduce a 1 was sent. This is repeated until the entire packet has been received. The sub-bit string is then decoded to extract the data. 10 = 1, 01 = 0, anything else must be erronous. Searching for the start bit only occurs a couple of ms before the next start bit is due to arrive reducing the chances of a accidental packet receive process being launched. 

each bit period is 0.8ms but this includes 2 sub-bits of the manchester code:
.                                                1                        0
.                                        --------                          --------
.                                                |                        |  
.                                                |                        |
.                                                |                        |
.                                                 --------        --------
.                                        <-----0.8ms------>       <-----0.8ms------> 
    
Mancester encoding is good for the IR receive modules, it's good for error detection, and hence this is why it was chosen. Currrently the packet period is 40ms, the half bit period is 400us. The start bit is 200/600us for ch 1, or 600/200us for ch 2, and all together there is 21 bits (16.8ms): 1 start, 8 Yaw_setpoint, 8 speed, 4 button bits. 2 channels are supported by interlacing packets for each robot between each other.

Use of Peripherals:
  - Timer0 - delay(), millis(), micros()
  - Timer1 - motor drive PWM
  - Timer2 - IR receiving and ESC PWM output.


