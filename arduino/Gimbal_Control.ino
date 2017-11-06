#include <Servo.h>

Servo panMotor;
Servo tiltMotor;

//the pan motor needs to be calibrated, instead of going from 9:00 to 3:00 it goes from 8:00 to 2:00
//to have the camera pointed straight ahead, set panAngle to 65 degrees
int panAngle=65; //180 rotates the gimbal counterclockwise, 0 rotates the gimbal clockwise, 65 centers it  
int tiltAngle=90; //180 tilts the gimbal up, 0 tilts the gimbal down

boolean commandReady=false;
int commandNum; 
int newAngle=0;

char command[2];
int data=0;
boolean newData=false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  panMotor.attach(3);
  tiltMotor.attach(5);

  panMotor.write(65);
  tiltMotor.write(90);
}

void loop() {
  // put your main code here, to run repeatedly:
  /*if(commandReady){
    commandReady =false;
    if (commandNum == 48){
      panCameraCounterClockwise();
    } else if (commandNum == 49){
      panCameraClockwise();
    } else if (commandNum == 50){
      tiltCameraUp();
    } else if (commandNum ==51){
      tiltCameraDown();
    } else if (commandNum == 52){
      Serial.print(getPanAngle());
    } else if (commandNum == 53){
      Serial.print(getTiltAngle());
    }
    commandNum=0;
  }*/  
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
/*
  Note: The serial monitor available on your computer will always convert each character
  into it's ASCII value and before sending. So if you want to use it to test
  commands you'll have to convert your bytes to ASCII characters first. 
  
  
  A command is 1 bytes long, the byte tells the program what function to run 
  00110000-panCameraCounterClockwise (ASCII value of 0)
  00110001-panCameraClockwise (ASCII value of 1)
  00110010-tiltCameraUp (ASCII value of 2)
  00110011-tiltCameraDown (ASCII value of 3)
  00110100-getPanAngle (ASCII value of 4)
  00110101-getTiltAngle (ASCII value of 5)
 */
void serialEvent() {
  while (Serial.available()) {
    commandNum= Serial.read();
    //Serial.println(commandNum);
    if (commandNum == 48){
      panCameraCounterClockwise();
    } else if (commandNum == 49){
      panCameraClockwise();
    } else if (commandNum == 50){
      tiltCameraUp();
    } else if (commandNum ==51){
      tiltCameraDown();
    } else if (commandNum == 52){
      Serial.print(getPanAngle());
    } else if (commandNum == 53){
      Serial.print(getTiltAngle());
    }
  }
}

void panCameraCounterClockwise () {
  if (panAngle < 180){
    panAngle=panAngle+5;
  }
  panMotor.write(panAngle);
}

void panCameraClockwise () {
  if (panAngle > 0){
    panAngle=panAngle-5; 
  }
  panMotor.write(panAngle);
}

void tiltCameraUp () {
  if (tiltAngle < 180){
    tiltAngle= tiltAngle+5;
  }
  tiltMotor.write(tiltAngle);
}

void tiltCameraDown () {
  if (tiltAngle > 0){
    tiltAngle= tiltAngle-5;
  }
  tiltMotor.write(tiltAngle);
}

int getPanAngle() {
  return panAngle;  
}

int getTiltAngle() {
  return tiltAngle;
}

