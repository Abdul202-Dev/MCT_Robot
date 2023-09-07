//In Motoren.cpp decide which Mode is on.
#include "MQTT.h"
#include "Motoren.h"
#include "Sensoren.h"
#include "Pixy.h"
#include "IRreciver.h"
#include "LCDdisplay.h"

volatile int IRcommand = 0;



extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
/*
   IR-Remoter entscheidet welcher Mode jetzt ist
*/

void setup() {
  Serial.begin(115200);

  SETUPMOTOREN();
  SETUPMQTT();
  SETUPSENSORES();
  SETUPPIXY();
  setupIRreciver();
  SETUPLCD();
}

void loop() {

  int ifIR = Get_irrecive();
  //Sve the recived value only, if it's an important Value
  if (ifIR != 0) {
    IRcommand = ifIR;
  }
  switch (IRcommand) {
    //Nr1(L)
    case 1:
      DeActive_M2();
      Printfirst("L ohne vermeidung");
      LOOPmotors_M1();
      RestartLOOPmotors_M1();
      break;
    //Nr2(L mit hinders vermeidung)
    case 2:
      Active_M2();
      Printfirst("L mit vermeidung");
      LOOPmotors_M1();
      RestartLOOPmotors_M1();
      break;
    //Nr3(pixy cam)
    case 3:
      DeActive_M2();
      Printfirst("Objekt folgen");
      LOOPFollow();

      break;
  }
  // Nr5(über IRremote Steuern =>bewegen)
  if (IRcommand > 5) {
    Printfirst("IR remote");
    LOOPIRremote(IRcommand);
    Serial.println(IRcommand);

  }

}
