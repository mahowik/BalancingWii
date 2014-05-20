
/*
 * Welcome to MultiWii.
 *
 * If you see this message, chances are you are using the Arduino IDE. That is ok.
 * To get the MultiWii program configured for your robot, you must switch to the tab named 'config.h'.
 * Maybe that tab is not visible in the list at the top, then you must use the drop down list at the right
 * to access that tab. In that tab you must enable your board or sensors and optionally various features.
 * For more info go to http://www.multiwii.com/wiki/index.php?title=Main_Page
 *
 * Have fun, and do not forget MultiWii is made possible and brought to you under the GPL License.
 *
 */
 

 
/***

########## Extended for balancing robot by Mahowik ############

BalancingWii rev 0.1 
=======
Hi all! 

This is the balancing robot based on modified/extended MultiWii 2.3 firmware.

Video: 
http://youtu.be/U8bBna9iWCU

Blogs:
http://forum.rcdesign.ru/blogs/83206/blog18515.html
http://www.multiwii.com/forum/viewtopic.php?f=7&t=4787


Hardware:
- Arduino nano (atmega328p)
- mpu6050 gyro-accelerometer (GY_521)
- any RC receiver with CPPM (ppmsum) output
- A4988 motor drivers
- nema 17 stepper motors
- 1/8 Buggy Wheels
- Buzzer 


Pinout for Arduino nano (atmega328p):

A0 - V_BATPIN: after the resistor divisor we should get [0V;5V]->[0;1023] on analogue V_BATPIN with R1=33k and R2=51k, 
     i.e. (+12v)~51k~(A0 pin)~33k~(GND)
A2 - BUZZERPIN

I2C:
A4 - SDA
A5 - SCL

RC control:
D2 - CPPM (PPM_SUM)

Motor driver pins:
D5 - STEP1 (PORTD 5)
D6 - STEP2 (PORTD 6)
D7 - DIR1 (PORTD 7)
D8 - DIR2 (PORTB 0)
D4 - ENABLE (for both)

If you look to the tail of the robot:
right motor = STEP1 & DIR1
left motor  = STEP2 & DIR2

   
Also see for new defines added with this project for robot setup:
    
  #define CURRENT_AXIS    PITCH       // possible to choose ROLL or PITCH axis as current.
  
  //#define INVERT_CURRENT_AXIS       // invert current axis sign, i.e. instead of turning sensor board
  
  //#define REVERSE_MOTORS_DIRECTION  // reverse both motors direction

  #define MAX_SPEED           400  // should be <= 500
  #define MAX_TARGET_ANGLE    120  // where 10 = 1 degree, should be <= 15 degree (i.e. <= 150) 
  #define MAX_STEERING        90   // should be <= 100 
  
  
  //#define GY_521_INVERTED_BY_Z  // Chinese 6  DOF with  MPU6050, LLC, inverted/reversed by Z
   

Big thanks for the projects:
- MultiWii - my favourite to start robotics and become in love with this!  :)
- B-Robot https://github.com/JJulio/b-robot    
   

Enjoy! ;)

***/
 
 

