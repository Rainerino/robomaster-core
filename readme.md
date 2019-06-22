Our guide to get started: https://docs.google.com/document/d/19cvIZilfux3r4_W7IBOViRhD_JSUb53r_CtSJ3eYy1E/edit?usp=sharing 

This project runs on Keil IDE. It is modified from the DJI official standard robot code.

Changes:

__Gimbal Branch__

2019.06.20 Added gimbal position based control. The yaw/pitch channels from the joystick are mapped to yaw/pitch positions of the gimbal. Gimbal initializes to center in this mode of control, and gimbal stops at the position.

2019.06.22 Receives data (ascii characters or int) from USART, then turns the gimbal to the designated position.


__Main Branch__

2019.06.19 Added USART receiving and transmitting. Board receives a character and transmitts char+1.
