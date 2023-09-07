#include <Wire.h>
#include <HCSR04.h>
#include <Adafruit_MCP23017.h>
#include <Arduino.h>
#ifndef Sensoren_FUNCTIONS_H
#define Sensoren_FUNCTIONS_H
extern void SETUPSENSORES();
extern float GetUL_SWert();
extern boolean  GetIR_L_SWert();
extern boolean GetIR_R_SWert();
extern boolean GetTaster_SWert();  

#endif
