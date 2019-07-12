__Intro__
Our guide to get started: https://docs.google.com/document/d/19cvIZilfux3r4_W7IBOViRhD_JSUb53r_CtSJ3eYy1E/edit?usp=sharing

This project runs on Keil IDE. It is modified from the DJI official standard robot code.



===========================================================================================================================================

__Main Branch__

__2019.06.19__ Added USART receiving and transmitting. Board receives a character and transmitts char+1.

__2019.07.12__ Merged with gimbal control branch (up to 2019.06.30 USART command shoot);
               Merged with swirl branch



===========================================================================================================================================

__Gimbal Branch__

Documentation for gimbal branch: https://docs.google.com/document/d/125cNgk1NbFqwFEMUcOG53Q9k7h3AweMsF87ulM5lq9g/edit?usp=sharing

__2019.06.20__ Added gimbal position based control. The yaw/pitch channels from the joystick are mapped to yaw/pitch positions of the gimbal. Gimbal initializes to center in this mode of control, and gimbal stops at the position.

__2019.06.22__ Receives data (ascii characters or int) from USART, then turns the gimbal to the designated position.

__2019.06.23__ USART incorporated with basic vision tracking. Gimbal turns at constant speed until tracked object is centered.

__2019.06.30__ USART command shoot. Send angle difference (still testing).







