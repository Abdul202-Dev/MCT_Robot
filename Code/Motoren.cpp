//In Motoren.cpp I'm moving the Robot debending on the results from the Functions in others files.
#include "Motoren.h"
#include "MQTT.h"
#include "Sensoren.h"
#include "LCDdisplay.h"

//#include <HCSR04.h>

//Forward [done]
//turn left(90°)[done]
//Turn right(90°)[Done]
//Auf dem Platz drehen(180°)[Done]
//logic for the fainal movement(main=>lop)[Do]
//Speed Percent (0-1023)
/*
  Send message to node-red
  (Subscribe,Publishe,Topic,Payload)[]
*/
//Ultraschal Sensor[done]
//IR[done]
/*
  (falls nicht klapt=>iDCM=iD*32)
  //Distance in cm[Try]
*/
//Fit the Speeds together[try]


#define FAST        400
#define SLOW        250

// Zugriff auf RTOS-Funktionalität ???
extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}

uint8_t dirR = 0, dirL = 0; // flags für Drehrichtung der Motoren

//uint8_t dirR = 0, dirL = 0; // flags für Drehrichtung der Motoren

//------------ Odometrie ------------------------------------
// variables to store the number of encoder pulses
volatile unsigned long leftCount = 0;
volatile unsigned long rightCount = 0;
volatile int iState1 = 0;
volatile int iRouteA = 0;
volatile int iSpeed = FAST;
volatile int iSpeedL = iSpeed;
volatile boolean bForward = 0;
volatile boolean bFRoad = 0;
volatile int iPLcount = 0;
volatile int iPRcount = 0;
volatile unsigned long iFinRoadL = 0;
volatile unsigned long iFinRoadR = 0;
volatile int i = 0;
volatile int iActiveM2 = 0;
volatile float fFDistance = 0;
volatile float fDSuppose = 0;
volatile boolean bIRL = 0;
volatile boolean bIRR = 0;
//to Save Previous encoders values
volatile int iPright = 0;
volatile int iPleft = 0;

//Motortreiber DRV8833 Pegel von 3-5V möglich
#define MOTORL_1    14           // pwm signal for motor controller left(Speed)(left)
#define MOTORL_2    27        // direction of motor left (back/forwurd)(left)
//right motor
#define MOTORR_1    26       // pwm signal for motor controller right(speed)(right)
#define MOTORR_2    25      // direction of motor right(back/forwurd)(Right)
// variables to store the number of encoder pulses


// pins for the encoder inputs
#define R_ENCODER_A 17
#define R_ENCODER_B 16
#define L_ENCODER_A 5
#define L_ENCODER_B 4
/*------------------ Variablen für PWM -----------------------
  (man braucht eine Channel für jedes motor(zu kontrollieren von geschwindigkeit )
*/
#define PWM_CHANNEL_0     0              // use first channel of 16 channels (started from zero)
#define PWM_CHANNEL_1     1              // use second channel of 16 channels (started from zero)
#define PWM_CHANNEL_2     2              // use third channel of 16 channels (started from zero)
#define PWM_TIMER_10_BIT  10             // use 10 bit precision for PWM timer
#define PWM_TIMER_8_BIT   8              // use 8 bit precision for Servo-Motor PWM timer
#define PWM_BASE_FREQ     1000           // use 1000 Hz as a PWM base frequency
#define PWM_SERVO_FREQ    166            // use 166 Hz as a Servo-Motor PWM base frequency
#define PWM_PIN_A         MOTORL_1       // motor one PIN GPIO25
#define PWM_PIN_B         MOTORR_1       // motor two PIN GPIO27

//------------------------ Prototypes ----------------------------
void pinDefs(void);
void initPWM_DC(void);
void setMotorSpeed(signed int, signed int);
void leftEncoderEvent(void);
void rightEncoderEvent(void);
void pinDefs(void);
void initPWM_DC(void);
void setMotorSpeed(signed int, signed int);
void leftEncoderEvent(void);
void rightEncoderEvent(void);

void SETUPMOTOREN()
{
  pinDefs();
  initPWM_DC();
  // initialize hardware interrupts
  attachInterrupt(digitalPinToInterrupt(R_ENCODER_A), rightEncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(L_ENCODER_A), leftEncoderEvent, CHANGE);
  setMotorSpeed(0, 0);

  Serial.print("erfolg Setup mottors");
}
//convert impuls into cm(35impuls==1cm)
void vConvertToCM() {

  int iDif = 0;
  String sDirection = "";
  if (iState1 == 0 || iState1 == 2) {
    sDirection = "vorwar";
  } else if (iState1 == 1) {
    sDirection = "Nach Links";
  } else if (iState1 == 3) {
    sDirection = "Nach Rechts";
  }
  iRouteA = leftCount / 37;
  Serial.print(iRouteA);
  Serial.println(sDirection);
  sDirection = String(iRouteA) + sDirection;
  //display the distance on the LCD
  Printsecond(sDirection.c_str());
  //Send the distance and direction
  Pub(sDirection);

}




//eRestart
void eRestart() {
  while (1) {
    leftCount = 0;
    rightCount = 0;
    if ((leftCount == 0) && (rightCount == 0)) {
      break;
    }
  }

}
//ePrint
void ePrint() {

  Serial.print("Left:");
  Serial.println(leftCount);

  Serial.print("right:");
  Serial.println(rightCount);
}

//change state(Mode1)
void cState() {
  if (iState1 == 0) {
    iState1 = 1;
  } else if (iState1 == 1) {
    iState1 = 2;
  } else if (iState1 == 2) {
    iState1 = 3;
  } else if (iState1 == 3) {
    iState1 = 4;
  } else if (iState1 == 4) {
    iState1 = 10;
  }
}
//Controll speed
void CSpeed()
{
  int Difl = leftCount - iPLcount;
  int Difr = rightCount - iPRcount;
  int iResult = 10;
  iPLcount = leftCount;
  iPRcount = rightCount;
  if (Difl > Difr) {
    iSpeedL -= iResult;
    iSpeed += iResult;
  }
  if (Difl < Difr) {
    iSpeed -= iResult;
    iSpeedL += iResult;
  }

}
//Reset Speed
void rSpeed()
{
  iSpeed = FAST;
  iSpeedL = iSpeed;
}

//endless forward
void forward() {
  Obtscale_Taster();
  //Controll rhe motors speed
  CSpeed();
  //both Motors forward
  setMotorSpeed(iSpeedL, iSpeed);
  //Forward is Stoping(to stop Controlling the left/rightCount)
vConvertToCM();
}


//forward for specific Distance
void forward(int iDistanceL, int iDistanceR) {
Obtscale_Taster();

  //both Motors forward reaching iDistances
  while (1)
  {
    //Controll rhe motors speed
    CSpeed();

    if (leftCount <= 4000000000 || rightCount <= 4000000000)
    {
      if (iActiveM2 != 2) {
        //Convert To CM
        vConvertToCM();
      }

    } else if (leftCount >= 4000000000 || rightCount >= 4000000000)
    {
      //Set encoders values on 0
      eRestart();
    }
    if (iActiveM2 == 2) {
      LOOPsensors_M2();
    }
    if ((iDistanceL > leftCount) && (iDistanceR > rightCount)) {
      setMotorSpeed(iSpeedL, iSpeed);
    } else if ((iDistanceL > leftCount) && (iDistanceR <= rightCount)) {
      setMotorSpeed(iSpeedL, 0);

    } else if ((iDistanceL <= leftCount) && (iDistanceR > rightCount)) {
      setMotorSpeed(0, iSpeed);

    } else if ((iDistanceR <= rightCount) && (iDistanceL <= leftCount)) {
      //Stop Motors
      setMotorSpeed(0, 0);
      Serial.println("finishing: forward");
      //Set encoders values on 0
      eRestart();
      //Change State
      cState();
      //Reset the speed
      rSpeed();
      break;
    }
  }
}

//backward
void backward() {
  //make the speed negative to change the direction
  int iSpeedR1 = -iSpeed;
  int iSpeedL1 = -iSpeedL;
  setMotorSpeed(iSpeedL1, iSpeedR1);
}



//Turn X° to left (depend on the value of the Parameter "iDistance")
void TurnL(int iDistanceL, int iDistanceR) {
  int iSpeed1 = -iSpeed;
  unsigned long iMAXD = ULONG_MAX - iDistanceL;
  eRestart();
    Printsecond("Links Drehen");


  //right Motor backward reaching "ULONG_MAX -iDistance" and left one forward reaching "iDistanceL"
  while (1) {
    if ((iDistanceR > rightCount) && ((iMAXD < leftCount) || (leftCount == 0))) {
      setMotorSpeed(iSpeed1, iSpeed);
    } else if ((iDistanceR <= rightCount) && (iMAXD < leftCount)) {
      setMotorSpeed(iSpeed1, 0 );

    } else if (((iDistanceR > rightCount) && (iMAXD >= leftCount))) {
      setMotorSpeed(0, iSpeed);

    } else if ((iDistanceR <= rightCount) && (iMAXD >= leftCount)) {
      setMotorSpeed(0, 0);
      //Set encoders values on 0
      eRestart();

      break;
    }
  }
}

//Turn 90° to the left
void TurnLeft () {
  //TurnL(440, 474);
   TurnRight(440, 470);

}
void Active_M2() {
  iActiveM2 = 2;
}
void DeActive_M2() {
  iActiveM2 = 0;
}
//Turn  to the right
void TurnRight(int iDistanceL, int iDistanceR) {
  int iSpeed1 = -iSpeed;
  unsigned long iMAXD = ULONG_MAX - iDistanceR;
  eRestart();
    Printsecond("Rechts Drehen");

  //right Motor backward reaching "ULONG_MAX -iDistance" and left one forward reaching "iDistanceL"
  while (1) {
    ePrint();
    if ((iDistanceL > leftCount) && ((iMAXD < rightCount) || (rightCount == 0))) {
      setMotorSpeed(iSpeed, iSpeed1);
    } else if ((iDistanceL <= leftCount) && (iMAXD < rightCount)) {
      setMotorSpeed(0, iSpeed1);

    } else if (((iDistanceL > leftCount) && (iMAXD >= rightCount))) {
      setMotorSpeed(iSpeed, 0);

    } else if ((iDistanceL <= leftCount) && (iMAXD >= rightCount)) {
      setMotorSpeed(0, 0);
      //Set encoders values on 0
      
      eRestart();
      Serial.println("finishing: right");

      break;
    }
  }
}

//Turn 90° to the right
void TurnoRightexact () {
  //TurnRight(447, 468);
    TurnRight(440, 470);

}
//Turn  to the right to avoid the obstacle
void TurnobstacleR () {
  TurnRight(451, 470);
}
//Turn to the left to avoid the obstacle
void TurnobstacleL () {

  TurnL(451, 470);
}
// turn 180° to left
void TurnAtPlace() {
  TurnL(885, 955);

}

//endless left2(for follow)
void TurnLeftPixy() {
  Serial.println("TurnLeft");

  //make the speed negative to change the direction
  int iSpeed1 = iSpeed + 100;

  //one Motor forward an the other backward
  setMotorSpeed(iSpeedL, iSpeed1);


}
//endless Right2(for follow)
void TurnRightPixy() {
  Serial.println("TurnRight");

  //make the speed negative to change the direction
  int iSpeedL1 = iSpeed + 100;

  //one Motor forward an the other backward
  setMotorSpeed(iSpeedL1, iSpeed);
  //Forward is Stoping(to stop Controlling the left/rightCount)


}
 void Obtscale_Taster (){
  //if there is an object very close to the Roboter, go backward
  if(GetTaster_SWert()){
    backward();
    delay(100);
  }
  setMotorSpeed(0,0);
 }
 void RestartLOOPmotors_M1(){
  iState1=0;
 }
void LOOPmotors_M1()
{
        setMotorSpeed(0, 0);
      delay(700);
      eRestart();
  while(iState1!=10){
     //logic of the Movement(State Machine)
  switch (iState1) {

    //forward 1m
    case 0:
      forward(3739, 3689);
      //Restart
      setMotorSpeed(0, 0);
      delay(700);
      eRestart();
      break;
    //Turn Left
    case 1: TurnLeft();
      //Restart
      setMotorSpeed(0, 0);
      delay(700);
      eRestart();
      //forward 1m
      forward(3739, 3689);
      //Restart
      setMotorSpeed(0, 0);
      delay(300);
      eRestart();

      break;
    case 2: TurnAtPlace();
      //Restart
      setMotorSpeed(0, 0);
      delay(700);
      eRestart();
      //forward 1m
      forward(3739, 3689);
      //Restart
      setMotorSpeed(0, 0);
      delay(700);
      eRestart();
      break;
    case 3: TurnoRightexact();
          //Restart
      setMotorSpeed(0, 0);
      delay(700);
      eRestart();
      //forward 1m
      forward(3739, 3689);
            //Restart
      setMotorSpeed(0, 0);
      delay(700);
      eRestart();
      //Reset the speed
          break;
    case 4: TurnAtPlace();
          //Restart
      setMotorSpeed(0, 0);
      delay(700);
      eRestart();
      iState1 = 10;
    default:  //Restart
      setMotorSpeed(0, 0);
      delay(700);
      eRestart();
      break;
   }
  }
 
}
//Avoid the obstacle, turning left
void goL_Avoiding() {

    //turn left
  TurnobstacleL();
  //restart encoders
  setMotorSpeed(0, 0);
  eRestart();
  delay(700);
  //Update the value of the IR_R sensor
  bIRR = GetIR_R_SWert();
  //go forward untile passing obstacle and save the distance
  //start recording the road
  do {
    //go Forward
    forward();
    //Update the value of the IR_R sensor
    bIRR = GetIR_R_SWert();

  } while (!bIRR);
  iFinRoadL = leftCount;
  iFinRoadR = rightCount;
  delay(280);
  setMotorSpeed(0, 0);
  delay(700);
 
  //restart encoders values
  eRestart();

  //turn right
  TurnoRightexact();
  //restart encoders
  eRestart();
  //go forward finding the right side of the obstacle
  do {
    //go Forward
    forward();
    //Update the value of the IR_R sensor
    bIRR = GetIR_R_SWert();
  } while (bIRR);

  //go forward passing the right side of the obstacle
  do {
    //go Forward
    forward();
    //Update the value of the IR_L sensor
    bIRR = GetIR_R_SWert();
  } while (!bIRR);

  delay(700);
  setMotorSpeed(0, 0);

  delay(700);
  eRestart();

  //after passing the obstacle go on the previous  road again
  TurnLeft();

  //restart encoders
  setMotorSpeed(0, 0);
  delay(1000);
  eRestart();

  //go forward finding the previous road
  forward(iFinRoadL, iFinRoadR);
  setMotorSpeed(0, 0);
  delay(700);
  eRestart();
  delay(700);
  eRestart();
  //Turn left
  TurnLeft();
  setMotorSpeed(0, 0);
  delay(700);
  eRestart();
  Serial.print("finish vermeiden L");
}
//Avoid the obstacle, turning Right
void goR_Avoiding() {
  //turn right
  TurnobstacleR();
  //restart encoders
  setMotorSpeed(0, 0);
  eRestart();
  delay(700);
  //Update the value of the IR_L sensor
  bIRL = GetIR_L_SWert();
  //go forward untile passing obstacle and save the distance
  //start recording the road
  do {
    //go Forward
    forward();
    //Update the value of the IR_L sensor
    bIRL = GetIR_L_SWert();

  } while (!bIRL);
  iFinRoadL = leftCount;
  iFinRoadR = rightCount;
  delay(280);
  setMotorSpeed(0, 0);
  delay(700);
 
  //restart encoders values
  eRestart();

  //turn Left
  TurnLeft();
  //restart encoders
  eRestart();
  //go forward finding the right side of the obstacle
  do {
    //go Forward
    forward();
    //Update the value of the IR_L sensor
    bIRL = GetIR_L_SWert();
  } while (bIRL);

  //go forward passing the right side of the obstacle
  do {
    //go Forward
    forward();
    //Update the value of the IR_L sensor
    bIRL = GetIR_L_SWert();
  } while (!bIRL);

  delay(700);
  setMotorSpeed(0, 0);

  delay(700);
  eRestart();

  //after passing the obstacle go on the previous  road again
  TurnLeft();

  //restart encoders
  setMotorSpeed(0, 0);
  delay(1000);
  eRestart();

  //go forward finding the previous road
  forward(iFinRoadL, iFinRoadR);
  setMotorSpeed(0, 0);
  delay(700);
  eRestart();
  delay(700);
  eRestart();
  //Turn right
  TurnoRightexact();
  setMotorSpeed(0, 0);
  delay(700);
  eRestart();
  Serial.print("finish vermeiden R");

}
void LOOPsensors_M2() {
  //Get Sensors Values
  fFDistance = GetUL_SWert();
  fDSuppose = 6.5;
  if (fFDistance <= fDSuppose && fFDistance > 0) {

    bIRL = GetIR_L_SWert();
    //IR_R is defect
    bIRR = GetIR_L_SWert();
    bIRR = 0;

    //to Save Previous encoders values
    iPright = rightCount;
    iPleft = leftCount;
    setMotorSpeed(0, 0);
    //restart encoders
    eRestart();
    //set "iFnRoad" on 0 (to save the road to find it again)
    iFinRoadR = 0;
    iFinRoadL = 0;

    //if there is also an obscale on the right side, go left
     if (!bIRR && bIRL) {
    goL_Avoiding();
    }
    //if there is no other obstacle or one on the left side, go to the right
    else {
    goR_Avoiding();
    }
    setMotorSpeed(0, 0);
    delay(1000);
    //set encoders on the previous values
    rightCount = iPright;
    leftCount = iPleft;
 

  }


}

// ===                 PIN Definitions IN/OUT                   ===
void pinDefs(void) {
  //pins to control Motors
  pinMode(MOTORL_1, OUTPUT);
  pinMode(MOTORL_2, OUTPUT);
  pinMode(MOTORR_1, OUTPUT);
  pinMode(MOTORR_2, OUTPUT);

  //input pins from encoders
  pinMode(L_ENCODER_A, INPUT);
  pinMode(L_ENCODER_B, INPUT);
  pinMode(R_ENCODER_A, INPUT);
  pinMode(R_ENCODER_B, INPUT);
}

// ===                   PWM Initialisierung                    ===
void initPWM_DC(void) {
  /*  (PWM_CHANNELs in dem Controller(Werte geben und pins bestimmen))
    Setup timer and attach timer to a PWM pin
  */
  //PWM_TIMER_10_BIT=1023 Different speeds, PWM_BASE_FREQ= frequence
  ledcSetup(PWM_CHANNEL_0, PWM_BASE_FREQ, PWM_TIMER_10_BIT);
  ledcSetup(PWM_CHANNEL_1, PWM_BASE_FREQ, PWM_TIMER_10_BIT);

  ledcAttachPin(PWM_PIN_A, PWM_CHANNEL_0);
  ledcAttachPin(PWM_PIN_B, PWM_CHANNEL_1);
}

// ===                   Motor-Control PWM===

void setMotorSpeed(signed int motor1, signed int motor2) {     // Motor gets speed through PWM here DRV8833 - L293, L298 etc.

  /* xIN1  xIN2  Function
      PWM   0    Forward PWM, fast decay
      1    PWM   Forward PWM, slow decay
      0    PWM   Reverse PWM, fast decay
      PWM   1    Reverse PWM, slow decay
  */
  //change direction if negative values
  if (motor1 < 0) {
    digitalWrite(MOTORL_2, 0);
    motor1 = -motor1;
    dirR = 0;
  }
  else {
    digitalWrite(MOTORL_2, 1);
    motor1 = 1023 - motor1;
    dirR = 1;
  }

  if (motor2 < 0) {
    digitalWrite(MOTORR_2, 0);
    motor2 = -motor2;
    dirL = 1;
  }
  else {
    digitalWrite(MOTORR_2, 1);
    motor2 = 1023 - motor2;
    dirL = 0;
  }
  //identify The speed
  if (motor1 > 1023) motor1 = 1023;  //10bit Maximum ist 1023!
  if (motor2 > 1023) motor2 = 1023;
  //send the speed values to the PWM_CHANNELS
  ledcWrite(PWM_CHANNEL_0, motor1);  // 10bit PWM, 1000Hz
  ledcWrite(PWM_CHANNEL_1, motor2);  // channel, duty cycle

}



// ===             Encoder Events for interrupt call by         ===
void leftEncoderEvent(void) {

  if (digitalRead(L_ENCODER_A) == HIGH) {
    if (digitalRead(L_ENCODER_B) == LOW) {
      leftCount--;
    } else {
      leftCount++;
    }
  } else {
    if (digitalRead(L_ENCODER_B) == LOW) {
      leftCount++;
    } else {
      leftCount--;
    }
  }
}
void rightEncoderEvent(void) {

  if (digitalRead(R_ENCODER_A) == HIGH) {
    if (digitalRead(R_ENCODER_B) == LOW) {
      rightCount--;
    } else {
      rightCount++;
    }
  } else {
    if (digitalRead(R_ENCODER_B) == LOW) {
      rightCount++;
    } else {
      rightCount--;
    }
  }
}
