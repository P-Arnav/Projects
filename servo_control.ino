#include <Servo.h>

Servo baseServo;  
Servo shoulderServo;
Servo elbowServo;
Servo gripServo;

int basePin = 9;
int shoulderPin = 10;
int elbowPin = 11;
int gripPin = 12;

void moveArmToRed();          
void moveArmToGreen();
void moveArmToBlue();
void moveArmToDefault();
void moveArmToPickup();
void moveServoSmooth(Servo &servo, int targetAngle, int delayTime);

void setup(){
  Serial.begin(9600);  
  baseServo.attach(basePin);
  shoulderServo.attach(shoulderPin);
  elbowServo.attach(elbowPin);
  gripServo.attach(gripPin);
  moveArmToDefault(); 
}

void loop(){
  while (Serial.available()){   
    char color = Serial.read();
    Serial.println(color); 

    if (color == 'R'){
      moveArmToPickup();
      moveArmToRed();
    } 
    else if (color == 'G'){
      moveArmToPickup();
      moveArmToGreen();
    } 
    else if (color == 'B'){
      moveArmToPickup();
      moveArmToBlue();
    }
    else if (color == 'D'){
      moveArmToDefault();
    }
  }
}

void moveArmToRed(){
  Serial.println("Moving to Red Object...");
  moveServoSmooth(shoulderServo, 140, 7);
  moveServoSmooth(elbowServo, 170, 7);
  delay(500);
  moveServoSmooth(baseServo, 45, 5);
  delay(500);
  moveServoSmooth(gripServo, 0, 2);
  delay(200);
}

void moveArmToGreen(){
  Serial.println("Moving to Green Object...");
  moveServoSmooth(shoulderServo, 140, 7);
  moveServoSmooth(elbowServo, 170, 7);
  delay(500);
  moveServoSmooth(baseServo, 150, 5);
  delay(500);
  moveServoSmooth(gripServo, 0, 2);
  delay(200);
}

void moveArmToBlue(){
  Serial.println("Moving to Blue Object...");
  moveServoSmooth(shoulderServo, 140, 10);
  moveServoSmooth(elbowServo, 170, 10);
  delay(500);
  moveServoSmooth(baseServo, 90, 5);
  delay(500);
  moveServoSmooth(gripServo, 0, 2);
  delay(200);
}

// Function to move the arm to default position
void moveArmToDefault(){
  Serial.println("Returning to Default Position...");
  moveServoSmooth(baseServo, 90, 5);
  moveServoSmooth(shoulderServo, 145, 10);
  moveServoSmooth(elbowServo, 120, 10);
  moveServoSmooth(gripServo, 90, 2);
}

void moveArmToPickup(){
  Serial.println("Moving to Pickup...");
  moveServoSmooth(shoulderServo, 140, 10);
  moveServoSmooth(elbowServo, 170, 10);
  moveServoSmooth(baseServo, 0, 5);
  moveServoSmooth(gripServo, 0, 5);
  moveServoSmooth(elbowServo, 130, 7);
  moveServoSmooth(shoulderServo, 180, 7);
  delay(500);
  moveServoSmooth(gripServo, 120, 2);
}

// Function to move servo smoothly
void moveServoSmooth(Servo &servo, int targetAngle, int delayTime){
  int currentAngle = servo.read();
  if (currentAngle < targetAngle){
    for (int i = currentAngle; i <= targetAngle; i++){
      servo.write(i);
      delay(delayTime);
    }
  } else {
    for(int i = currentAngle; i >= targetAngle; i--){
      servo.write(i);
      delay(delayTime);
    }
  }
}
