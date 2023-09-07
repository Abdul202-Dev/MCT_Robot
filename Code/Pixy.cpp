#include "Pixy.h"
#include "Motoren.h"
#include "Sensoren.h"
#include "Pixy2I2C.h"


Pixy2I2C pixy;




volatile int iBlock = 0;
volatile float fSpaceM = 15;
volatile float fSpaceI = 0;
volatile uint16_t X_Location;
volatile uint16_t  iSig = 0;
volatile boolean  bSBackward = 0;
volatile int iSpeedPT = 300;
volatile int iSpeedPT1 = -iSpeedPT;



void SETUPPIXY()
{
  pixy.init();
  Serial.print("Pixy Initialized...\n");
}
void fnCenterOb(uint16_t X_Location) {

  Serial.println(X_Location);
  setMotorSpeed(0, 0);
  //if the Object is on the Right, turn Left
  if (X_Location < 100) {
    if (bSBackward) {
      setMotorSpeed(iSpeedPT, iSpeedPT1);
    } else {
      TurnLeftPixy();
    }



  }  //if the Object is on the Left, turn Right
  if (X_Location > 190) {
    if (bSBackward) {
      setMotorSpeed(iSpeedPT1, iSpeedPT);

    } else {
      TurnRightPixy();
    }

  }
}

void LOOPFollow()
{

  // grab blocks
  pixy.ccc.getBlocks();

  // If there are detect blocks,Start a loop to get evrey block
  if (pixy.ccc.numBlocks)
  {
    for (iBlock = 0; iBlock < pixy.ccc.numBlocks; iBlock++)
    {
      //Get the x_Location of the object to take the right action
      fSpaceI = GetUL_SWert();
      //Get X_location and Sig
      iSig = pixy.ccc.blocks[iBlock].m_signature;
      X_Location = pixy.ccc.blocks[iBlock].m_x;
      //if it is the 12th sig, follow the object
      if (iSig == 10) {
        //Center the object, if isn't in the Center
        if ((X_Location > 190) || (X_Location < 100)) {
          fnCenterOb(X_Location);
          //update X_Location
          pixy.ccc.blocks[iBlock].print();

          Serial.println(X_Location);

        }

        if (fSpaceI > fSpaceM) {
          //go forward
          forward();
          bSBackward = false;
        } else if ((fSpaceI <= fSpaceM) && (fSpaceI > fSpaceM - 2)) {
          //Stop
          setMotorSpeed(0, 0);
          bSBackward = true;
        } else if (fSpaceI < fSpaceM) {
          //go backward to make the fSpaceI==fSpaceM
          backward();
          bSBackward = true;
        }
      } else {
        setMotorSpeed(0, 0);
      }

    }
  } else {
    setMotorSpeed(0, 0);
  }
}
