#include "IRreciver.h"
#include "Motoren.h"
#include <IRrecv.h>
#include <IRutils.h>
#include "LCDdisplay.h"

#define TurnSpeed 300

#define M1 16753245
#define M2 16736925
#define M3 16769565
#define M4 16720605

#define IRforward 16718055
#define IRright 16734885
#define IRbackward 16730805
#define IRleft 16716015
#define IRstop 16726215

#define IRFehler 18446744073709551615
volatile int iState3 = 0;
volatile int Speed=TurnSpeed;
volatile int Speed1=-Speed;




// An IR detector/demodulator is connected to GPIO pin 14(D5 on a NodeMCU
// board).
const uint16_t kRecvPin = 19;

IRrecv irrecv(kRecvPin);

decode_results results;

void setupIRreciver() {
  irrecv.enableIRIn();  // Start the receiver

}
int Get_irrecive() {
  if (irrecv.decode(&results)) {
    uint64_t iRrecive = results.value;

    if (iRrecive != IRFehler) {

    }
    irrecv.resume();  // Receive the next value

    if (iRrecive == M1) {
      return 1;

    } else if (iRrecive == M2) {
      return 2;

    } else if (iRrecive == M3) {
      return 3;

    } else if (iRrecive == M4) {
      return 4;

    } else if (iRrecive == IRforward) {
      return 6;

    } else if (iRrecive == IRright) {
      return 7;

    } else if (iRrecive == IRbackward) {
      return 8;

    } else if (iRrecive == IRleft) {
      return 9;

    } else if (iRrecive == IRstop) {
      return 10;

    }
  }
  return 0;
}
void LOOPIRremote(int iState3) {
 

  switch (iState3) {
      //Turn Right
    case 6: 
    Printsecond("IR vorwaer");
    forward();
      break;
      //Turn Right
    case 7: 
    Printsecond("IR rechts");
    setMotorSpeed(Speed,Speed1);
      break;
      //Backward
    case 8:
    Printsecond("IR ruckwaer");
    backward();
      break;
      //Turn Left
    case 9:
    Printsecond("IR Links");
   setMotorSpeed(Speed1,Speed);
      break;
      //Stop Motors
    case 10:
    Printsecond("IR stop");
    setMotorSpeed(0,0);
    eRestart();
      break;
  }
}
