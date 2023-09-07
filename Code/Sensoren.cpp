#include "Sensoren.h"
#include "Motoren.h"


#define TASTER_R 4
#define TASTER_L 5

//IR Sensors
#define IR_L 0
#define IR_R 1

//ultrasonic sensor
#define Triger  32
#define Echo  18

//Adafruit_MCP23017 object
Adafruit_MCP23017 io1;

// Initialize Ultra sensor 
UltraSonicDistanceSensor distanceSensor(Triger, Echo);

//Setup sensors
void SETUPSENSORES() {
  //to have access on the pins(1) from MCP23017
  io1.begin(1);
  //define pins
  io1.pinMode(IR_L, INPUT);
  io1.pinMode(IR_R, INPUT);
  
  io1.pinMode(TASTER_R, INPUT); // configuration for TasterR
  io1.pullUp(TASTER_R, HIGH);   // turn on a 100K pullup internally
  
  io1.pinMode(TASTER_L, INPUT); // configuration for TasterL
  io1.pullUp(TASTER_L, HIGH);   // turn on a 100K pullup internally
}
float GetUL_SWert() {
  return distanceSensor.measureDistanceCm();
}
boolean GetIR_L_SWert() {
  return io1.digitalRead(IR_L);
}
boolean GetIR_R_SWert() {
  return io1.digitalRead(IR_R);

}
boolean GetTaster_SWert() {
  if(!(io1.digitalRead(TASTER_R)) || !(io1.digitalRead(TASTER_L))){
  return true;
}else{
  return false;
}
 }
