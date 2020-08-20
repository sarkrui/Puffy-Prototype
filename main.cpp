#include "Arduino.h"
#include <Wire.h>
#include "Adafruit_MPR121.h"
#include <Servo.h>

#ifndef _BV
#define _BV(bit) (1 << (bit)) 
#endif

#define enterThreshold -4
#define leaveThreshold 4

#define filteredEnterThreshold -3
#define filteredLeaveThreshold 2


#define startPin 1
#define sensePins 4

#define MAX 2500
#define MIN 500

#define inflatePin A1
#define deflatePin A0

#define servoUpdateDelay 0      //speed of servoupdater (millis between steps)

Adafruit_MPR121 cap = Adafruit_MPR121();

uint16_t lasttouched = 0;
uint16_t currtouched = 0;

void senseTouchPoints();
void fullDebugWindow();
void setDestinationPositions(int);
void moveServo();
void updatePositions();
void behavior();
void inflate(int power);
void deflate(int power);
void bothOff();


int prevFilteredData = 0;

int prevSensVal[sensePins+startPin] = {};
int prevSensValFiltered[sensePins+startPin] = {};
bool senseHover[sensePins+startPin] = {};

int destinServoPos[6] = {};     //final destination position
int moveServoPos[6] = {};       //move to this position now
Servo servo[6];

//7 positions X 6 servos's
 int finalPosition[7][6]={
  {145, 35, 90, 90, 35, 145},
  {165, 50, 50, 165, 110, 110},
  {90, 90, 35, 145, 145, 35},
  {50, 165, 110, 110, 165, 50},
  {35, 145, 145, 35, 90, 90},
  {110, 110, 165, 50, 50, 165},
  {90, 90, 90, 90, 90, 90}
 };


long prevServoUpdateMillis;
int selectedPosition; //holds 1-7 
int prevSelectedPosition;


int behaviorCase;

long prevBehaviorTimer;

int irrDeflateCounter = 0;

bool didTimer = false;


bool debug = true;
void setup() {
  Serial.begin(9600);

  while (!Serial) { // needed to keep leonardo/micro from starting too fast!
    delay(10);
  }
  
  Serial.println("Adafruit MPR121 Capacitive Touch sensor test"); 
  
  // Default address is 0x5A, if tied to 3.3V its 0x5B
  // If tied to SDA its 0x5C and if SCL then 0x5D
  if (!cap.begin(0x5A)) {
    Serial.println("MPR121 not found, check wiring?");
    while (1);
  }
  Serial.println("MPR121 found!");


  //set initial values
  for(int i = startPin; i < (sensePins+startPin); i++){
    senseHover[i] = 0;
    prevSensVal[i] = 0;
  }



  //servo 
  servo[0].attach(3, MIN, MAX);   //change MIN & MAX to calibrate servo
  servo[1].attach(5, MIN, MAX);  
  servo[2].attach(6, MIN, MAX);
  servo[3].attach(9, MIN, MAX);
  servo[4].attach(10, MIN, MAX);
  servo[5].attach(11, MIN, MAX);


  //pumps
  pinMode(inflatePin, OUTPUT);
  pinMode(deflatePin, OUTPUT);

  
}

void loop() {

  int sumFilter = 0;
  bool touched = false;

  //set to old values
  for(int i = startPin; i < (sensePins+startPin); i++){
    if(((int) cap.baselineData(i) - (int) prevSensVal[i]) >= leaveThreshold){
      if((int) cap.filteredData(i) - (int) prevSensValFiltered[i] >= filteredLeaveThreshold){
        senseHover[i] = 0;
      }
    }
    if(((int) cap.baselineData(i) - (int) prevSensVal[i]) <= enterThreshold){
      if((int) cap.filteredData(i) - (int) prevSensValFiltered[i] <= filteredEnterThreshold){
        senseHover[i] = 1;
      }
    }

    if(senseHover[1] == 1) selectedPosition = 5;
    else if(senseHover[2] == 1) selectedPosition = 1;
    else if(senseHover[3] == 1) selectedPosition = 2;
    else if(senseHover[4] == 1) selectedPosition = 4;
    else selectedPosition = 6;  //return to center


    
  



    //selectedPosition = 3;                                               //?!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


    if(debug){
    
    Serial.print((int) cap.baselineData(i));
    Serial.print("-");
    Serial.print((int) prevSensVal[i]);
    Serial.print("=");
    Serial.print((int) cap.baselineData(i) - (int) prevSensVal[i]);
    Serial.print(" && ");

    Serial.print((int) cap.filteredData(i));
    Serial.print("-");
    Serial.print((int) prevSensValFiltered[i]);
    Serial.print("=");
    Serial.print((int) cap.filteredData(i) - (int) prevSensValFiltered[i]);
    Serial.print("=>");
    Serial.print(senseHover[i]);
    Serial.print("\t");
        Serial.print("\t");

    }

    prevSensVal[i] = (int) cap.baselineData(i);
    prevSensValFiltered[i] = (int) cap.filteredData(i);
  }



    if(selectedPosition == 6 && prevSelectedPosition != 6){
      behaviorCase = 2;
      irrDeflateCounter = random(5, 8);
    }
    else if(selectedPosition == 6 && irrDeflateCounter < 1){
      behaviorCase = 0;
    }
    else if(selectedPosition != 6){
      behaviorCase = 1;
    }

    if(selectedPosition == 6 && irrDeflateCounter > 1 && didTimer == true){
      behaviorCase = 3;
    }



    prevSelectedPosition = selectedPosition;

    behavior();

  //update servos
  setDestinationPositions(selectedPosition);
  updatePositions();
  moveServo();

    // for(int i = 0; i < 5; i++){
    //   Serial.print(moveServoPos[i]);
    //   Serial.print("->");
    //   Serial.print(destinServoPos[i]);
    //   Serial.print("\t");
    // }


    if(debug){
   Serial.print("\t");
   Serial.print("||");
   Serial.print("\t");
   Serial.print(selectedPosition);
      Serial.print("\t");
   Serial.print(behaviorCase);
         Serial.print("\t");
   Serial.print(prevSelectedPosition);
            Serial.print("\t");
   Serial.print(irrDeflateCounter);
               Serial.print("\t");
   Serial.print(millis() - prevBehaviorTimer);

   Serial.println();
  }
  
  // put a delay so it isn't overwhelming
  delay(10);
}


void behavior(){
  switch(behaviorCase){
    case 0:                 //idle / breathing centrally
      if(millis() - prevBehaviorTimer < 1000){
        inflate(50);
      }
      else if(millis() - prevBehaviorTimer > 1000 && millis() - prevBehaviorTimer < 2000){
        deflate(200);
      }
      else{
        bothOff();
      }
     
      if(millis() - prevBehaviorTimer > 3500){
        prevBehaviorTimer = millis();
      }

    break;
    
    
    case 1: //inflate
      inflate(50);
      didTimer = false;
      prevBehaviorTimer = millis();
      
    break;

    case 2: //wait for deflationj
      if(millis() - prevBehaviorTimer > 5000){      //left alone for irregular deflation
        behaviorCase = 3;
        prevBehaviorTimer = millis();
        didTimer = true;
      }    
      bothOff();

    break;

    case 3:
      if(millis() - prevBehaviorTimer < 200){
        deflate(20);
      }
      else if(millis() - prevBehaviorTimer > 200 && millis() - prevBehaviorTimer < 600){
        bothOff();
      }
      else if(millis() - prevBehaviorTimer > 605){
        irrDeflateCounter--;

        prevBehaviorTimer = millis();
      }




      //return to idle position
      if(irrDeflateCounter < 1){
        behaviorCase = 0;
      }
              
      break;


  }
}

void bothOff(){
  digitalWrite(inflatePin, 0);
  digitalWrite(deflatePin, 0);
}

void inflate(int power){
      digitalWrite(inflatePin, power);
      digitalWrite(deflatePin, 0);
}

void deflate(int power){
      digitalWrite(inflatePin, 0);
      digitalWrite(deflatePin, power);
}


void moveServo(){
  int calib[6] = {5, 5, 10, 0, 0, 0};
 
  servo[0].write(       moveServoPos[0]     +calib[0]);
  servo[1].write(180-   moveServoPos[1]      +calib[1]);
  servo[2].write(       moveServoPos[2]      +calib[2]);
  servo[3].write(180-   moveServoPos[3]      +calib[3]);
  servo[4].write(       moveServoPos[4]      +calib[4]);
  servo[5].write(180-   moveServoPos[5]     +calib[5]);

}

void updatePositions(){
 if(millis() - prevServoUpdateMillis > servoUpdateDelay){
    for(int i = 0; i < 6; i++){
      if(moveServoPos[i] < destinServoPos[i]){
        moveServoPos[i] = moveServoPos[i] + 3;
      }
      if(moveServoPos[i] > destinServoPos[i]){
        moveServoPos[i] = moveServoPos[i] - 3;
      }
      else{
      }

    }

    prevServoUpdateMillis = millis();
 }

}

void setDestinationPositions(int servoPos){
  for(int i = 0; i < 6; i++){
    destinServoPos[i] = finalPosition[selectedPosition][i];
  }
}


void senseTouchPoints(){
  // Get the currently touched pads
  currtouched = cap.touched();
  
  for (uint8_t i=0; i<12; i++) {
    // it if *is* touched and *wasnt* touched before, alert!
    if ((currtouched & _BV(i)) && !(lasttouched & _BV(i)) ) {
      Serial.print(i); Serial.println(" touched");
    }
    // if it *was* touched and now *isnt*, alert!
    if (!(currtouched & _BV(i)) && (lasttouched & _BV(i)) ) {
      Serial.print(i); Serial.println(" released");
    }
  }

  // reset our state
  lasttouched = currtouched;
}



void fullDebugWindow(){
  // debugging info, what
  Serial.print("Pin: ");
  Serial.print("\t");
  for(uint8_t i = 0; i < 12; i++){
    Serial.print(i); Serial.print("\t");
  }
  Serial.println();
    Serial.print("Filt: ");
    Serial.print("\t");
  for (uint8_t i=0; i<12; i++) {
    Serial.print(cap.filteredData(i)); Serial.print("\t");
  }
  Serial.println();
  Serial.print("Base: ");
      Serial.print("\t");

  for (uint8_t i=0; i<12; i++) {
    Serial.print(cap.baselineData(i)); Serial.print("\t");
  }
  Serial.println();
  Serial.println();
}