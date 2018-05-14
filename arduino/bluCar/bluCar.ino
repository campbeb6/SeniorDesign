/*.....
 * For car to go straight, continuously send 
 */

#include<SPI.h>


byte byteRead;
//LED definitions
const int ledFrontTop = 31;
const int ledFrontBott = 30;
const int ledBackTop = 32;
const int ledBackBott = 33;

//motor definitions
const int maxSpeedL = 255;
const int maxSpeedR = 235;
const int startSpeedL = 255;
const int startSpeedR = 255;
const int deltSpL = 30;
const int deltSpR = 30;

const int leftMotorSensor = 2;
const int rightMotorSensor = 3; 

const int pinMotorR = 5;
const int pinMotorL = 9;
const int pinMotorRR = 6;
const int pinMotorLL = 10;

unsigned long leftWheelTime, rightWheelTime;
long leftMotorCount, rightMotorCount;

void setup(){
  
  //motor pins
  pinMode(pinMotorR, OUTPUT);
  pinMode(pinMotorL, OUTPUT);
  pinMode(pinMotorRR, OUTPUT);
  pinMode(pinMotorLL, OUTPUT);
  pinMode(leftMotorSensor,INPUT);
  pinMode(rightMotorSensor,INPUT);
  attachInterrupt(0, leftWheelCnt, CHANGE);
  attachInterrupt(1, rightWheelCnt, CHANGE);
  leftMotorCount = 0;
  rightMotorCount = 0;
  
  Serial.begin(9600);

  /*
   * LEDs Setup
   */
  pinMode(ledFrontTop, OUTPUT);
  pinMode(ledFrontBott, OUTPUT);
  pinMode(ledBackTop, OUTPUT);
  pinMode(ledBackBott, OUTPUT);

  delay(200);
}

int motorSpeedL;
int motorSpeedR;

void loop(void){
  readData();
  digitalWrite(7,HIGH);
  direct();
}

/*
 * RF Functions
 */
void readData() {
  if (Serial.available()) {
    byteRead = Serial.read();
    //read the data and store it in byteRead

    showData();
  }
}

void showData() {
  //Serial.print("Data received: ");
  //Serial.print(dataReceived[0]);
  //Serial.println();
}

/*
 * Car Functions
 */

boolean carStopped = false;

void direct() {
  if(byteRead == '3') {
    straight();
  }
  if(byteRead == '4') {
    left();
  }
  else if(byteRead == '5') {
    right();
  }
  else if(byteRead == '6') {
    stopCar();
  }
  else if(byteRead == '0') {
    digitalWrite(ledFrontTop, HIGH);
    digitalWrite(ledFrontBott, HIGH);
    digitalWrite(ledBackTop, LOW);
    digitalWrite(ledBackBott, LOW);
  }
  else if(byteRead == '1') {
    digitalWrite(ledFrontTop, LOW);
    digitalWrite(ledFrontBott, LOW);
    digitalWrite(ledBackTop, HIGH);
    digitalWrite(ledBackBott, HIGH);
  }
  else if(byteRead = '2') {
    digitalWrite(ledFrontTop, LOW);
    digitalWrite(ledFrontBott, LOW);
    digitalWrite(ledBackTop, LOW);
    digitalWrite(ledBackBott, LOW);
  }
}

void straight() {
  
  attachInterrupt(0, leftWheelCnt, CHANGE);
  attachInterrupt(1, rightWheelCnt, CHANGE);
  reset();
  
  if(leftMotorCount > rightMotorCount) {
    //slow down left motor
    motorSpeedL = maxSpeedL - deltSpL;
    motorSpeedR = maxSpeedR;
  }
  
  else if(rightMotorCount > leftMotorCount) {
    //slow down right motor
    motorSpeedR = maxSpeedR - deltSpR;
    motorSpeedL = maxSpeedL;
  }
  
  else {
    motorSpeedL = maxSpeedL;
    motorSpeedR = maxSpeedR;
  }

  //update speed
  analogWrite(pinMotorL, motorSpeedL);
  analogWrite(pinMotorR, motorSpeedR);
  
}

void right() {
  
  detachInterrupt(0);
  detachInterrupt(1);
  reset();
  motorSpeedL = maxSpeedL;
  motorSpeedR = 0;
  
  analogWrite(pinMotorL, motorSpeedL);
  analogWrite(pinMotorR, motorSpeedR);
}

void left() {
  
  detachInterrupt(0);
  detachInterrupt(1);
  reset();
  motorSpeedL = 0;
  motorSpeedR = maxSpeedR;
  
  analogWrite(pinMotorL, motorSpeedL);
  analogWrite(pinMotorR, motorSpeedR);
}

void stopCar() {
  
  detachInterrupt(0);
  detachInterrupt(1);
  motorSpeedL = 0;
  motorSpeedR = 0;
  carStopped = true;
  
  analogWrite(pinMotorL, motorSpeedL);
  analogWrite(pinMotorR, motorSpeedR);
}

void reset() {
  if(carStopped) {
   analogWrite(pinMotorL, maxSpeedL);
   analogWrite(pinMotorR, maxSpeedR);
   delay(50);
   carStopped = false;
   leftMotorCount = 0;
   rightMotorCount = 0;
  }
}

void leftWheelCnt()
{
  unsigned long tempTime = micros();
  if (tempTime > leftWheelTime + 1000) {
    leftWheelTime = tempTime;
    leftMotorCount++;
  }
}

void rightWheelCnt() {
   unsigned long tempTime = micros();
   if (tempTime > rightWheelTime + 1000) {
    rightWheelTime = tempTime;
    rightMotorCount++;
   }
}
