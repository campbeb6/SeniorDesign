/*.....
 * For car to go straight, continuously send 
 */

#include<SPI.h>
#include<RF24.h>

//ce, csn pin
RF24 radio(4, 8);

char dataReceived[1];

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
  Serial.println("Begin");

  /*
   * LEDs Setup
   */
  pinMode(ledFrontTop, OUTPUT);
  pinMode(ledFrontBott, OUTPUT);
  pinMode(ledBackTop, OUTPUT);
  pinMode(ledBackBott, OUTPUT);

  /*
   * Setup for RF
   */
  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(0x75);         //must match Pi
  radio.openWritingPipe(0xF0F0F0F0E1LL);
  const uint64_t pipe = 0xE8E8F0F0E1LL;
  radio.openReadingPipe(1, pipe);
  radio.enableDynamicPayloads();
  radio.powerUp();
  radio.startListening();

  //analogWrite(pinMotorL, startSpeedL);
  //analogWrite(pinMotorR, startSpeedR);
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
  if (radio.available()) {
    //read the data and store it in dataReceived
    radio.read(&dataReceived, sizeof(dataReceived));
    //show the read data
    showData();
  }
}

void showData() {
  Serial.print("Data received: ");
  Serial.print(dataReceived[0]);
  Serial.println();
}

/*
 * Car Functions
 */

boolean carStopped = false;

void direct() {
  if(dataReceived[0] == 'a') {
    straight();
  }
  if(dataReceived[0] == 'b') {
    left();
  }
  else if(dataReceived[0] == 'c') {
    right();
  }
  else if(dataReceived[0] == 'd') {
    stopCar();
  }
  else if(dataReceived[0] == 'e') {
    digitalWrite(ledFrontTop, HIGH);
    digitalWrite(ledFrontBott, LOW);
    digitalWrite(ledBackTop, LOW);
    digitalWrite(ledBackBott, LOW);
  }
  else if(dataReceived[0] == 'f') {
    digitalWrite(ledFrontTop, LOW);
    digitalWrite(ledFrontBott, LOW);
    digitalWrite(ledBackTop, HIGH);
    digitalWrite(ledBackBott, LOW);
  }
  else if(dataReceived[0] = 'g') {
    digitalWrite(ledFrontTop, LOW);
    digitalWrite(ledFrontBott, LOW);
    digitalWrite(ledBackTop, LOW);
    digitalWrite(ledBackBott, LOW);
  }
  /*else if(dataReceived[0] == 'f') {
    digitalWrite(ledFrontTop, LOW);
    digitalWrite(ledFrontBott, HIGH);
    digitalWrite(ledBackTop, LOW);
    digitalWrite(ledBackBott, LOW);
  }
  else if(dataReceived[0] == 'g') {
    digitalWrite(ledFrontTop, LOW);
    digitalWrite(ledFrontBott, LOW);
    digitalWrite(ledBackTop, HIGH);
    digitalWrite(ledBackBott, LOW);
  }
  else if(dataReceived[0] == 'h') {
    digitalWrite(ledFrontTop, LOW);
    digitalWrite(ledFrontBott, LOW);
    digitalWrite(ledBackTop, LOW);
    digitalWrite(ledBackBott, HIGH);
  } */ 
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
