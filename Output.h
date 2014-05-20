#ifndef OUTPUT_H_
#define OUTPUT_H_

extern uint8_t DRIVER_PIN[5];

void initOutput();
void mixTable();
void writeMotors();
void setMotorSpeed(uint8_t motorNum, int16_t tspeed);

#endif /* OUTPUT_H_ */
