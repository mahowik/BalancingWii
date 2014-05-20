#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "Alarms.h"

/**************************************************************************************/
/***************                  Motor driver pins                ********************/
/**************************************************************************************/
#if defined(PROMINI)
  uint8_t DRIVER_PIN[5] = {5,6,7,8,4};   //STEP1 (PORTD 5), STEP2 (PORTD 6), DIR1 (PORTD 7), DIR2 (PORTB 0), ENABLE
#endif
#if defined(PROMICRO)
#endif
#if defined(MEGA)
#endif

/**************************************************************************************/
/************  Calculate and writes the motors values                ******************/
/**************************************************************************************/

uint16_t periodsCounter[2];      // counters for periods
uint16_t subPeriod[2][8];        // eight subperiodPaddings 
uint8_t subPeriodIndex[2];       // index for subperiodPaddings

#define ZERO_SPEED  65535
#define MAX_ACCEL   4


// Divided into 8 sub-periods to increase the resolution at high speeds (short periods)
// subperiodPadding = ((1000 % vel)*8)/vel;
void calculateSubperiods(uint8_t motor) {

  uint8_t subperiodPadding;
  uint16_t absSpeed;
  uint8_t i;

  if (actualMotorSpeed[motor] == 0) {
    for (i=0; i<8; i++) {
      subPeriod[motor][i] = ZERO_SPEED;
    }  
    return;
  }
  
  #ifdef REVERSE_MOTORS_DIRECTION
    actualMotorDir[motor] = (actualMotorSpeed[motor] > 0) ? 1 : 0; 
  #else
    actualMotorDir[motor] = (actualMotorSpeed[motor] > 0) ? 0 : 1; 
  #endif  
  
  absSpeed = abs(actualMotorSpeed[motor]);

  subPeriod[motor][0] = 1000/absSpeed;
  for (i=1; i<8; i++) {
    subPeriod[motor][i] = subPeriod[motor][0];
  }  
  // Calculate the sub-period padding. 
  subperiodPadding = ((1000 % absSpeed)*8)/absSpeed;
  if (subperiodPadding > 0) {
    subPeriod[motor][1]++;
  }  
  if (subperiodPadding > 1) {
    subPeriod[motor][5]++;
  }  
  if (subperiodPadding > 2) {
    subPeriod[motor][3]++;
  }  
  if (subperiodPadding > 3) {
    subPeriod[motor][7]++;
  }  
  if (subperiodPadding > 4) {
    subPeriod[motor][0]++;
  }  
  if (subperiodPadding > 5) {
    subPeriod[motor][4]++;
  }  
  if (subperiodPadding > 6) {
    subPeriod[motor][2]++;
  }  
}

void setMotorSpeed(uint8_t motorNum, int16_t speed) {

  speed = constrain(speed, -MAX_SPEED, MAX_SPEED); 

  // LIMIT MAX ACCELERATION
  int16_t acceleration = speed - actualMotorSpeed[motorNum];
  if (acceleration > MAX_ACCEL) {
    actualMotorSpeed[motorNum] += MAX_ACCEL;
  } else if (acceleration < -MAX_ACCEL) {
    actualMotorSpeed[motorNum] -= MAX_ACCEL;
  } else {
    actualMotorSpeed[motorNum] = speed;
  }
  
  calculateSubperiods(motorNum);  // We use four subperiodPaddings to increase resolution

  // To save energy when its not running...
  if ((actualMotorSpeed[0] == 0) && (actualMotorSpeed[1] == 0)) {
    digitalWrite(4,HIGH);   // Disable motors
  } else {
    digitalWrite(4,LOW);   // Enable motors
  }  
}


#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))

ISR(TIMER1_COMPA_vect) // DRIVER_PIN[5] = {5,6,7,8,4};   //STEP1 (PORTD 5), STEP2 (PORTD 6), DIR1 (PORTD 7), DIR2 (PORTB 0), ENABLE
{
  periodsCounter[0]++;
  periodsCounter[1]++;
  
  if (periodsCounter[0] >= subPeriod[0][subPeriodIndex[0]]) {
    periodsCounter[0] = 0;
    
    if (subPeriod[0][0] != ZERO_SPEED) {
      if (actualMotorDir[0]) {
        SET(PORTD,7);  // DIR Motor 1
      } else {
        CLR(PORTD,7);
      }  
      // We need to wait at lest 200ns to generate the Step pulse...
      subPeriodIndex[0] = (subPeriodIndex[0]+1)&0x07; // subPeriodIndex from 0 to 7
      
      SET(PORTD,5); // STEP Motor 1
      delayMicroseconds(1);
      CLR(PORTD,5);
    }
  }
  
  if (periodsCounter[1] >= subPeriod[1][subPeriodIndex[1]]) {
    periodsCounter[1] = 0;
    
    if (subPeriod[1][0] != ZERO_SPEED) {
    
      if (actualMotorDir[1]) {
        SET(PORTB,0);   // DIR Motor 2
      } else {
        CLR(PORTB,0);
      }  
      subPeriodIndex[1] = (subPeriodIndex[1]+1)&0x07;
      
      SET(PORTD,6); // STEP Motor 1
      delayMicroseconds(1);
      CLR(PORTD,6);
    }
  }
}



/**************************************************************************************/
/************        Initialize the Timers and Registers         ******************/
/**************************************************************************************/
void initOutput() {
  /****************            mark all driver pins as Output             ******************/
  for(uint8_t i=0;i<5;i++) {
    pinMode(DRIVER_PIN[i],OUTPUT);
  }

  /********  Specific Timers & Registers for the atmega328P (Promini)   ************/
#if defined(PROMINI)

  digitalWrite(4,HIGH);   // Disable motors

  //We are going to overwrite the Timer1 to use the stepper motors

  // STEPPER MOTORS INITIALIZATION
  // TIMER1 CTC MODE
  TCCR1B &= ~(1<<WGM13);
  TCCR1B |=  (1<<WGM12);
  TCCR1A &= ~(1<<WGM11); 
  TCCR1A &= ~(1<<WGM10);

  // output mode = 00 (disconnected)
  TCCR1A &= ~(3<<COM1A0); 
  TCCR1A &= ~(3<<COM1B0); 

  // Set the timer pre-scaler
  // Generally we use a divider of 8, resulting in a 2MHz timer on 16MHz CPU
  TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (2<<CS10);

  //OCR1A = 125;  // 16Khz
  //OCR1A = 100;  // 20Khz
  OCR1A = 80;   // 25Khz
  TCNT1 = 0;

  TIMSK1 |= (1<<OCIE1A);  // Enable Timer1 interrupt
  digitalWrite(4, LOW);   // Enable stepper drivers

#endif

  /****************  Specific Timers & Registers for the MEGA's    ******************/
#if defined(MEGA)
#endif

  /******** Specific Timers & Registers for the atmega32u4 (Promicro)   ************/
#if defined(PROMICRO)
#endif

}

