#ifndef Motoren_FUNCTIONS_H
#define Motoren_FUNCTIONS_H

#include <Arduino.h>
#include <PIDLoop.h>
#include <Wire.h>


extern void TurnRightPixy();
extern void TurnLeftPixy();
extern void pinDefs(void);
extern void initPWM_DC(void);
extern void setMotorSpeed(signed int, signed int);
extern void leftEncoderEvent(void);
extern void rightEncoderEvent(void);
extern void SETUPMOTOREN();
extern void vConvertToCM();
extern void eRestart();
extern void ePrint();
extern void cState();
extern void CSpeed();
extern void rSpeed();
extern void forward();
extern void forward(int,int);
extern void backward();
extern void TurnL(int, int);
extern void TurnLeft ();
extern void TurnRight(int, int);
extern void TurnoRightexact();
extern void TurnobstacleR();
extern void TurnobstacleL();
extern void TurnAtPlace();
extern void LOOPmotors_M1();
extern void LOOPsensors_M2();
extern void Active_M2();
extern void DeActive_M2();
extern void Obtscale_Taster();
extern void RestartLOOPmotors_M1();
#endif
