#include <Servo.h>

#define Ultrasonic 12
#define EnableServo 13
#define BuzzerPin 4
#define ButtonPin 2
#define Red 3
#define Green 5
#define Blue 6


Servo Lleg;  // create servo object to control a servo
Servo Rleg;
Servo Lfoot;
Servo Rfoot;
Servo Neck;

int RFcenter = 80;    // variable to store the center servo position
int LLcenter = 80;
int RLcenter = 80;
int LFcenter = 80;
int Neckcenter = 90;
int obstacleDistance = 0;
int obstacleLeft = 0;
int obstacleRight = 0;
int presentDistance = 0;
int tAngle = 30; //tilt angle
int uAngle = 35; //turn angle
int sAngle = 35; //swing angle
const int pingPin = 12;

void setup() {
  // initialize serial communication:
  Serial.begin(19200);

  Lleg.attach(7);  // attaches the servo on pin x to the servo object
  Rleg.attach(10);  // attaches the servo on pin x to the servo object
  Lfoot.attach(8);  // attaches the servo on pin x to the servo object
  Rfoot.attach(9);  // attaches the servo on pin x to the servo object
  Neck.attach(11);  // attaches the servo on pin x to the servo object

  pinMode(EnableServo,OUTPUT);
  digitalWrite(EnableServo,HIGH); //this turns on the power to the servos
  CenterServos(); //center the servos
  delay(500);
  digitalWrite(EnableServo,LOW); //turn power off after centering

  pinMode(Red, OUTPUT);
  digitalWrite(Red, LOW);
  pinMode(Blue, OUTPUT);
  digitalWrite(Blue, LOW);
  pinMode(Green, OUTPUT);
  digitalWrite(Green, LOW);

    pinMode(BuzzerPin, OUTPUT);
  digitalWrite(BuzzerPin, LOW);
  //Buzzer.PlayMelody();

  pinMode(ButtonPin, INPUT);
  digitalWrite(ButtonPin, HIGH); //pull up activated

  Serial.print("Ready");

  //while (digitalRead(ButtonPin) != LOW){
    //do nothing until the button pressed
  //}
    BuzzerBeep();
  for(int i=0;i<5;++i){
    delay(1000);
  }
    BuzzerBeep();
}

void loop()
{
  //check for obstacles, if none, go Forward, if found, turn the other way
  NeckLeft(1,30);
  NeckRight(1,30);
  CheckObstacle();
}

void CheckDistance(){
    // establish variables for duration of the ping,
  // and the distance result in inches and centimeters:
  long duration, cm;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);

  // convert the time into a distance

  cm = microsecondsToCentimeters(duration);
  obstacleDistance = cm;
}

long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

void CheckObstacle(){
  int Obstacle=0;
  CheckDistance();
  if (obstacleDistance >= 20){
    Obstacle=0;
  }
  if (obstacleDistance <= 20){ //check sensor
    obstacleDistance = presentDistance;
    BuzzerBeep();
    Neck.write(Neckcenter+30); //turn head left
    delay(100);
    CheckDistance();
    delay(100);
    obstacleDistance = obstacleLeft;
    Neck.write(Neckcenter-30); //turn head right
    delay(200);
    CheckDistance();
    delay(100);
    Neck.write(Neckcenter); //turn head to the center
    delay(100);
    obstacleDistance = obstacleRight;
    if (presentDistance >= obstacleLeft){
    Obstacle=Obstacle+1;
    }
    else if (presentDistance >= obstacleRight && obstacleLeft >= presentDistance){
      Obstacle=Obstacle+2;
    }
    else if (obstacleRight == obstacleLeft) {
    Obstacle=Obstacle+3;
    }
    else {
      Obstacle=Obstacle+3;
    }
  }
  switch (Obstacle){
    case 0: //no object
      digitalWrite(Green, HIGH);
      digitalWrite(Red, HIGH);
      Forward(1,30); //one step Forward
      digitalWrite(Green, LOW);
      digitalWrite(Red, LOW);
      break;
    case 1: //object on Left
      digitalWrite(Green, HIGH);
      TurnRight(2,30);
      digitalWrite(Green, LOW);
      break;
    case 2: //object on Right
      digitalWrite(Blue, HIGH);
      TurnLeft(2,30);
      digitalWrite(Blue, LOW);
      break;
    case 3: //obect in Front (both Left and Right detect the object)
      digitalWrite(Red, HIGH);
      TurnLeft(4,30); //turn around
      digitalWrite(Red, LOW);
      break;
  }
}

void CenterServos() // center the servos on powerup
{
    Lleg.write(LLcenter);              // tell servo to go to position in variable 'center'
    Rleg.write(RLcenter);              // tell servo to go to position in variable 'center'
    Lfoot.write(LFcenter);              // tell servo to go to position in variable 'center'
    Rfoot.write(RFcenter);              // tell servo to go to position in variable 'center'
    Neck.write(Neckcenter);              // tell servo to go to position in variable 'center'
    delay(100);                     // waits 100ms for the servos to reach the position
}

void BuzzerBeep(){ //beep the buzzer on event
    digitalWrite(BuzzerPin, HIGH);
    delay(100);
    digitalWrite(BuzzerPin, LOW);
}

//Servo Primatives

void TiltRightUp(byte ang, byte sp){
  //tilt right up
  for (int i=0; i<=ang; i+=5){
    Lfoot.write(LFcenter+i);
    Rfoot.write(RFcenter+i);
    delay(sp);
  }
}
void TiltRightDown(byte ang, byte sp){
  //tilt right down
  for (int i=ang; i>0; i-=5){
    Lfoot.write(LFcenter+i);
    Rfoot.write(RFcenter+i);
    delay(sp);
  }
}

void TiltLeftUp(byte ang, byte sp){
  //tilt left up
  for (int i=0; i<=ang; i+=5){
    Lfoot.write(LFcenter-i);
    Rfoot.write(RFcenter-i);
    delay(sp);
  }
}
void TiltLeftDown(byte ang, byte sp){
  //tilt left down
  for (int i=ang; i>0; i-=5){
    Lfoot.write(LFcenter-i);
    Rfoot.write(RFcenter-i);
    delay(sp);
  }
}

void LeftFootUp(char ang, byte sp){
  //tilt left up
  for (int i=0; i<=ang; i+=5){
    Lfoot.write(LFcenter-i);
    delay(sp);
  }
}
void LeftFootDown(byte ang, byte sp){
  //tilt left down
  for (int i=ang; i>0; i-=5){
    Lfoot.write(LFcenter-i);
    delay(sp);
  }
}

void RightFootUp(byte ang, byte sp){
  //tilt right up
  for (int i=0; i<=ang; i+=5){
    Rfoot.write(RFcenter+i);
    delay(sp);
  }
}
void RightFootDown(byte ang, byte sp){
  //tilt right down
  for (int i=ang; i>0; i-=5){
    Rfoot.write(RFcenter+i);
    delay(sp);
  }
}

void SwingRight(byte ang, byte sp){
  //swing right
  for (int i=0; i<=ang; i+=5){
    Lleg.write(LLcenter-i);
    Rleg.write(RLcenter-i);
    delay(sp);
  }
}
void SwingRcenter(byte ang, byte sp){
  //swing r->center
  for (int i=ang; i>0; i-=5){
    Lleg.write(LLcenter-i);
    Rleg.write(RLcenter-i);
    delay(sp);
  }
}

void SwingLeft(byte ang, byte sp){
  //swing left
  for (byte i=0; i<=ang; i=i+5){
    Lleg.write(LLcenter+i);
    Rleg.write(RLcenter+i);
    delay(sp);
  }
}
void SwingLcenter(byte ang, byte sp){
  //swing l->center
  for (byte i=ang; i>0; i=i-5){
    Lleg.write(LLcenter+i);
    Rleg.write(RLcenter+i);
    delay(sp);
  }
}

void RightLegIn(byte ang, byte sp){
  //swing right
  for (int i=0; i<=ang; i+=5){
    Rleg.write(RLcenter-i);
    delay(sp);
  }
}
void RightLegIcenter(byte ang, byte sp){
  //swing r->center
  for (int i=ang; i>0; i-=5){
    Rleg.write(RLcenter-i);
    delay(sp);
  }
}

void RightLegOut(byte ang, byte sp){
  //swing right
  for (int i=0; i<=ang; i+=5){
    Rleg.write(RLcenter+i);
    delay(sp);
  }
}
void RightLegOcenter(byte ang, byte sp){
  //swing r->center
  for (int i=ang; i>0; i-=5){
    Rleg.write(RLcenter+i);
    delay(sp);
  }
}

void LeftLegIn(byte ang, byte sp){
  //swing left
  for (byte i=0; i<=ang; i=i+5){
    Lleg.write(LLcenter+i);
    delay(sp);
  }
}
void LeftLegIcenter(byte ang, byte sp){
  //swing l->center
  for (byte i=ang; i>0; i=i-5){
    Lleg.write(LLcenter+i);
    delay(sp);
  }
}

void LeftLegOut(byte ang, byte sp){
  //swing left
  for (byte i=0; i<=ang; i=i+5){
    Lleg.write(LLcenter-i);
    delay(sp);
  }
}
void LeftLegOcenter(byte ang, byte sp){
  //swing l->center
  for (byte i=ang; i>0; i=i-5){
    Lleg.write(LLcenter-i);
    delay(sp);
  }
}

void NeckLeft(byte ang, byte sp){
  //swing left
  for (int i=0; i<=ang; i+=5){
    Neck.write(Neckcenter+i);
    delay(sp);
  }
}
void NeckRight(byte ang, byte sp){
  //swing right
  for (int i=0; i<=ang; i+=5){
    Neck.write(Neckcenter-i);
    delay(sp);
  }
}
void NeckIcenter(byte ang, byte sp){
  //swing neck->center
  for (byte i=ang; i>0; i=i-5){
    Neck.write(Neckcenter+i);
    delay(sp);
  }
}
void NeckOcenter(byte ang, byte sp){
  //swing neck->center
  for (byte i=ang; i>0; i=i-5){
    Neck.write(Neckcenter-i);
    delay(sp);
  }
}

//Walk Functions

void Forward(byte Steps, byte Speed){
  digitalWrite(EnableServo,HIGH);
  TiltRightUp(tAngle, Speed);
  for (byte j=0; j<Steps; ++j){
    SwingRight(sAngle, Speed);
    TiltRightDown(tAngle, Speed);
    TiltLeftUp(tAngle, Speed);
    SwingRcenter(sAngle, Speed);
    SwingLeft(sAngle, Speed);
    TiltLeftDown(tAngle, Speed);
    TiltRightUp(tAngle, Speed);
    SwingLcenter(sAngle, Speed);
  }
  TiltRightDown(tAngle, Speed);
  digitalWrite(EnableServo,LOW);
}

void TurnLeft(byte Steps, byte Speed){
  digitalWrite(EnableServo,HIGH);
  TiltLeftUp(uAngle, Speed);
  delay(20);
  for (byte j=0; j<Steps; ++j){
    LeftLegIn(sAngle, Speed);
    TiltLeftDown(uAngle, Speed);
    TiltRightUp(uAngle, Speed);
    delay(20);
    LeftLegIcenter(sAngle, Speed);
    RightLegOut(sAngle, Speed);
    TiltRightDown(uAngle, Speed);
    TiltLeftUp(uAngle, Speed);
    delay(20);
    RightLegOcenter(sAngle, Speed);
  }
  TiltLeftDown(uAngle, Speed);
  digitalWrite(EnableServo,LOW);
}

void TurnRight(byte Steps, byte Speed){
  digitalWrite(EnableServo,HIGH);
  TiltRightUp(uAngle, Speed);
  delay(20);
  for (byte f=0; f<=Stps; ++f){
    RightLegIn(sAngle, Speed);
    TiltRightDown(uAngle, Speed);
    TiltLeftUp(uAngle, Speed);
    delay(20);
    RightLegIcenter(sAngle, Speed);
    LeftLegOut(sAngle, Speed);
    TiltLeftDown(uAngle, Speed);
    TiltRightUp(uAngle, Speed);
    delay(20);
    LeftLegOcenter(sAngle, Speed);
  }
  TiltRightDown(uAngle, Speed);
  digitalWrite(EnableServo,LOW);
}
