#include <math.h>
#include <Servo.h>

typedef struct _Leg {
    Servo servoUL;
    Servo servoLL;

    int   posUL;
    int   posLL;

    void attach(int ul, int ll) {
        servoUL.attach(ul);
        servoLL.attach(ll);
    }

    void write(int ul, int ll) {
        servoUL.write(ul);
        servoLL.write(ll);
    }
} Leg;

Leg lFront; // left front leg
Leg rFront; // right front leg
Leg lRear; // left rear leg
Leg rRear; // right rear leg

Servo hipFL; // hip front left
Servo hipFR; // hip front right
Servo hipRL; // hip rear left
Servo hipRR; // hip rear right

//-------------------- LEG VARIABLES --------------------//

int LFUL = 0; // left front upper leg
int LFLL = 0; // left front lower leg

int LRUL = 0; // left rear upper leg
int LRLL = 0; // left rear lower leg

int RFUL = 0; // right front upper leg
int RFLL = 0; // right front lower leg

int RRUL = 0; // right rear upper leg
int RRLL = 0; // right rear lower leg

float xRF = 0; // right front x-coordinate
float xLF = 0; // left front x-coordinate
float xRR = 0; // right rear x-coordinate
float xLR = 0; // left rear x-coordinate

float yRF = -0.25; // right front y-coordinate
float yLF = -0.25; // left front y-coordinate
float yRR = -0.25; // right rear y-coordinate
float yLR = -0.25; // left rear y-coordinate

float xRFcalib = 0; // right front leg x-coordinate calibration
float xLFcalib = -0.05; // left front leg x-coordinate calibration
float xRRcalib = 0; // right rear leg x-coordinate calibration
float xLRcalib = 0; // left rear leg x-coordinate calibration

float yRFcalib = 0; // right front leg y-coordinate calibration
float yLFcalib = 0; // left front leg y-coordinate calibrati
float yRRcalib = 0; // right rear leg y-coordinate calibration
float yLRcalib = 0; // left rear leg y-coordinate calibration

float finalLF = 0; // y-coordinate + y-coordinate calibration value
float finalLR = 0; // y-coordinate + y-coordinate calibration value
float finalRF = 0; // y-coordinate + y-coordinate calibration value
float finalRR = 0; // y-coordinate + y-coordinate calibration value

int currentLoop = 0; // determines which leg should move in the walking gait

int walkSpeed = 0.003; // value for dx (change in x)

//-------------------- VARIABLES FOR IK --------------------//

float desiredQ1RF = 0; // desired angles for right front leg
float desiredQ2RF = 0;

float desiredQ1LF = 0; // desired angles for left front leg
float desiredQ2LF = 0;

float desiredQ1RR = 0; // desired angles for right rear leg
float desiredQ2RR = 0;

float desiredQ1LR = 0; // desired angles for left rear leg
float desiredQ2LR = 0;

const float a1 = 0.17; // upper leg bone length
const float a2 = 0.2; // lower leg bone length

#define PI 3.14159265

void setup() {
  Serial.begin(9600);

  Serial.println("aaa");

  rRear.attach(8, 9);
  lRear.attach(6, 7);
  lFront.attach(2, 3);
  rFront.attach(5, 4);

  hipFR.attach(10);
  hipFL.attach(11);
  hipRR.attach(12);
  hipRL.attach(13);
  
  xLF = -0.0667; // center of gait
  xLR = 0.0387; // upper domain (intercept between y=-0.25 and gait equation)
  xRF = -0.0667; // center of gait
  xRR = -0.1721; // lower domain (intercept between y=-0.25 and gait equation)
}

void loop() {
  
  Serial.println("BBB");

  yRFcalib = -0.01;
  yLFcalib = -0.04;
  yRRcalib = 0;
  yLRcalib = -0.05;

  while(currentLoop == 0){
    yLF = -(sq(3*xLF+0.2))-0.15; // gait
    yRR = -(sq(3*xRR+0.2))-0.15;

    yLR = -0.25;
    yRF = -0.25;

    xLF += walkSpeed;
    xRR += walkSpeed;
    xLR -= walkSpeed;
    xRF -= walkSpeed;

    finalLF = yLF+yLFcalib;
    finalLR = yLR+yLRcalib;
    finalRF = yRF+yRFcalib;
    finalRR = yRR+yRRcalib;

    Serial.println(finalLR);

    ikRF(xRF, finalRF); // calculates angles for each leg
    ikRR(xRR, finalRR);
    ikLF(xLF, finalLF);
    ikLR(xLR, finalLR);
    
    lRear.write(LRUL, LRLL);
    rRear.write(RRUL, RRLL);
    lFront.write(LFUL, LFLL);
    rFront.write(RFUL, RFLL);
    
    hipFL.write(55);
    hipFR.write(130);
    hipRL.write(127);
    hipRR.write(50);
    
    if (xLF >= 0.0387) {
      currentLoop += 1;
      Serial.println("1a");
    }
  }
  while(currentLoop == 1){
    yRR = -(sq(3*xRR+0.2))-0.15; // gait
    yRF = -(sq(3*xRF+0.2))-0.15;

    yLF = -0.25;
    yLR = -0.25;

    xRR += walkSpeed;
    xRF += walkSpeed;
    xLF -= walkSpeed;
    xLR -= walkSpeed;

    finalLF = yLF+yLFcalib;
    finalLR = yLR+yLRcalib;
    finalRF = yRF+yRFcalib;
    finalRR = yRR+yRRcalib;

    Serial.println(finalLR);

    ikRF(xRF, finalRF); // calculates angles for each leg
    ikRR(xRR, finalRR);
    ikLF(xLF, finalLF);
    ikLR(xLR, finalLR);
  
    lRear.write(LRUL, LRLL);
    rRear.write(RRUL, RRLL);
    lFront.write(LFUL, LFLL);
    rFront.write(RFUL, RFLL);
    
    hipFL.write(55);
    hipFR.write(130);
    hipRL.write(127);
    hipRR.write(50);

    if (xRR >= 0.0387) {
      currentLoop += 1;
      Serial.println("2a");
    }
  }
  while(currentLoop == 2){
    yRF = -(sq(3*xRF+0.2))-0.15; // gait
    yLR = -(sq(3*xLR+0.2))-0.15;

    yRR = -0.25;
    yLF = -0.25;

    xRF += walkSpeed;
    xLR += walkSpeed;
    xRR -= walkSpeed;
    xLF -= walkSpeed;

    finalLF = yLF+yLFcalib;
    finalLR = yLR+yLRcalib;
    finalRF = yRF+yRFcalib;
    finalRR = yRR+yRRcalib;

    Serial.println(finalLR);

    ikRF(xRF, finalRF); // calculates angles for each leg
    ikRR(xRR, finalRR);
    ikLF(xLF, finalLF);
    ikLR(xLR, finalLR);

    lRear.write(LRUL, LRLL);
    rRear.write(RRUL, RRLL);
    lFront.write(LFUL, LFLL);
    rFront.write(RFUL, RFLL);
    
    hipFL.write(55);
    hipFR.write(130);
    hipRL.write(127);
    hipRR.write(50);
    
    if (xRF >= 0.0387) {
      currentLoop += 1;
      Serial.println("3a");
    }
  }
  while(currentLoop == 3){
    yLR = -(sq(3*xLR+0.2))-0.15; // gait
    yLF = -(sq(3*xLF+0.2))-0.15;

    yRF = -0.25;
    yRR = -0.25;

    xLR += walkSpeed;
    xLF += walkSpeed;
    xRF -= walkSpeed;
    xRR -= walkSpeed;

    finalLF = yLF+yLFcalib;
    finalLR = yLR+yLRcalib;
    finalRF = yRF+yRFcalib;
    finalRR = yRR+yRRcalib;

    Serial.println(finalLR);

    ikRF(xRF, finalRF); // calculates angles for each leg
    ikRR(xRR, finalRR);
    ikLF(xLF, finalLF);
    ikLR(xLR, finalLR);

    lRear.write(LRUL, LRLL);
    rRear.write(RRUL, RRLL);
    lFront.write(LFUL, LFLL);
    rFront.write(RFUL, RFLL);
    
    hipFL.write(55);
    hipFR.write(130);
    hipRL.write(127);
    hipRR.write(50);

    if (xLR >= 0.0387) {
      currentLoop -= 3;
      Serial.println("4a");
    }
  }


}

//------------------------------------------------ FUNCTIONS ------------------------------------------------//

void ikRF(float xFuncRF, float yFuncRF){ // inverse kinematics for right front leg
  desiredQ2RF = acos((sq(xFuncRF) + sq(yFuncRF) - sq(a1) - sq(a2)) / (2 * a1 * a2));
  desiredQ1RF = atan2(yFuncRF, xFuncRF) - atan2((a2 * sin(desiredQ2RF)), (a1 + a2 * cos(desiredQ2RF)));

  RFLL = 180*(255-(180-((desiredQ2RF*180)/PI)))/255;
  
  if(desiredQ1RF <= 0){
    RFUL = 180*(255-(-((desiredQ1RF*180)/PI)))/255;
  }else{
    RFUL = 180*(75+(desiredQ1RF*180/PI))/255;
  }

  return RFUL, RFLL;
}

void ikRR(float xFuncRR, float yFuncRR){ // inverse kinematics for right rear leg
  desiredQ2RR = acos((sq(xFuncRR) + sq(yFuncRR) - sq(a1) - sq(a2)) / (2 * a1 * a2));
  desiredQ1RR = atan2(yFuncRR, xFuncRR) - atan2((a2 * sin(desiredQ2RR)), (a1 + a2 * cos(desiredQ2RR)));

  RRLL = 180*(255-(180-((desiredQ2RR*180)/PI)))/255;

  if(desiredQ1RR <= 0){
    RRUL = 180*(255-(-((desiredQ1RR*180)/PI)))/255;
  }else{
    RRUL = 180*(75+(desiredQ1RR*180/PI))/255;
  }

  return RRUL, RRLL;
}

void ikLF(float xFuncLF, float yFuncLF){ // inverse kinematics for left front leg
  desiredQ2LF = acos((sq(xFuncLF) + sq(yFuncLF) - sq(a1) - sq(a2)) / (2 * a1 * a2));
  desiredQ1LF = atan2(yFuncLF, xFuncLF) - atan2((a2 * sin(desiredQ2LF)), (a1 + a2 * cos(desiredQ2LF)));

  LFLL = 180*(180-((desiredQ2LF*180)/PI))/255;

  if(desiredQ1LF <= 0){
    LFUL = 180*(-((desiredQ1LF*180)/PI))/255;
  }else{
    LFUL = 180*(75+(desiredQ1LF*180/PI))/255;
  }

  return LFUL, LFLL;
}

void ikLR(float xFuncLR, float yFuncLR){ // inverse kinematics for left rear leg
  desiredQ2LR = acos((sq(xFuncLR) + sq(yFuncLR) - sq(a1) - sq(a2)) / (2 * a1 * a2));
  desiredQ1LR = atan2(yFuncLR, xFuncLR) - atan2((a2 * sin(desiredQ2LR)), (a1 + a2 * cos(desiredQ2LR)));

  LRLL = 180*(180-((desiredQ2LR*180)/PI))/255;

  if(desiredQ1LR <= 0){
    LRUL = 180*(-((desiredQ1LR*180)/PI))/255;
  }else{
    LRUL = 180*(75+(desiredQ1LR*180/PI))/255;
  }

  return LRUL, LRLL;
}

