# Project Overview
This is the Arduino code for the Gyro Motor Controller (GMC): A gyro stabilised mixer for small DC powered robots. The GMC allows for signals to be received either by conventional RF by connecting it to an RF receiver, or via Infra-Red by attachment of a IR receiver module that directly mounts on the board. 

# Features:
 - Automatic detection of RF or IR connectivity
 - Automatic detection of 2 channel, 3 channel, or [PPM] in the case of RF control
 - 1S or 2S LiPo low battery detection and ultra-low power shutdown
 - PID algorithm to reach the Yaw_setpoint as fast as possible - can be tuned on the fly via IR
 - A PCM output channel to control a servo or ESC when using IR
 - 2 IR channels supported on the same IR frequency

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
