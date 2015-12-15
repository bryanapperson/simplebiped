#include <Servo.h>
#include <ChibiOS_AVR.h>

MUTEX_DECL(lockMutex);
MUTEX_DECL(serialMutex);

#define EnableServo 3
#define BuzzerPin 13
#define ButtonPin 2
#define Red 6
#define Green 5
#define Blue 4


Servo Lleg;  // create servo object to control a servo
Servo Rleg;
Servo Lfoot;
Servo Rfoot;
Servo Neck;

#define EnableServo 13
#define BuzzerPin 4
#define ButtonPin 2
#define Red 3
#define Green 5
#define Blue 6

int RFcenter = 80;    // variables to store the center servo positions
int LLcenter = 80;
int RLcenter = 80;
int LFcenter = 80;
int Neckcenter = 90;
// Setup variables to store sensor readings
int obstacleDistance = 0;
int obstacleLeft = 0;
int obstacleCenter = 0;
int obstacleRight = 0;
int presentDistance = 0;
// declare reaction distances on object preception
int obstacleAhead = 20;
int obstacleWarning = 10;
int obstacleAlert = 8;
volatile int Obstacle = 0;
// declare angle values for walking
int tAngle = 25; //tilt angle
int uAngle = 35; //turn angle
int sAngle = 30; //swing angle
int neckAngle = 30; //angle for meck turn
const int pingPin = 12; // define sensor pin

// remember thread pointers
Thread* tp1;
Thread* tp2;

//------------------------------------------------------------------------------
// thread 1 - high priority for walking motion
// 200 byte stack beyond task switch and interrupt needs
static WORKING_AREA(waThread1, 200);

static msg_t Thread1(void *arg) {
  while (TRUE) {
  WalkDirection();
  }
}

//------------------------------------------------------------------------------
// thread 2 - scan for obstacles as walking
// 200 byte stack beyond task switch and interrupt needs
static WORKING_AREA(waThread2, 200);

static msg_t Thread2(void *arg) {
  while (TRUE) {
    ScanObstacle();
  }
  // end task
}
//------------------------------------------------------------------------------

void setup() {
  // initialize serial communication:
  Serial.begin(19200);

  // read any input
  delay(200);
  while (Serial.read() >= 0) {}

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

  Serial.print("Ready... ");

  chBegin(mainThread);
  // chBegin never returns, main thread continues with mainThread()
  // shouldn't return
  while(1) {}
}
//------------------------------------------------------------------------------
// main thread runs at NORMALPRIO
void mainThread() {

  // start walk thread
  tp1 = chThdCreateStatic(waThread1, sizeof(waThread1),
                          NORMALPRIO + 2, Thread1, NULL);

  // start object scan thread
  tp2 = chThdCreateStatic(waThread2, sizeof(waThread2),
                          NORMALPRIO + 2, Thread2, NULL);
}
//------------------------------------------------------------------------------
void loop() {
 // not used
}

void CheckDistance(){
    // establish variables for duration of the ping,
  // and the distance result in inches and centimeters:
  long duration, cm;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  chThdSleepMilliseconds(2);
  digitalWrite(pingPin, HIGH);
  chThdSleepMilliseconds(5);
  digitalWrite(pingPin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);

  // convert the time into a distance
  chThdSleepMilliseconds(10);
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

void ScanObstacle(){
  Neck.write(Neckcenter);
  chThdSleepMilliseconds(100);
  CheckDistance();
  if (obstacleDistance > 20){ //no obstacle nearby
    chMtxLock(&lockMutex);
    Obstacle=0;
    chMtxUnlock();
    chMtxLock(&serialMutex);
    Serial.print(obstacleDistance);
    Serial.print("cm center over 20");
    Serial.println();
    chMtxUnlock();
  }
  if (obstacleDistance <= obstacleAhead){ //check sensor
    BuzzerBeep();
    Neck.write(Neckcenter);
    chThdSleepMilliseconds(100);
  digitalWrite(Red, HIGH);
  CheckDistance();
  chThdSleepMilliseconds(10);
  obstacleCenter = obstacleDistance;
  Serial.print(obstacleCenter);
  Serial.print("cm center");
  Serial.println();
  Neck.write(Neckcenter+neckAngle); //turn head left
  chThdSleepMilliseconds(200);
  digitalWrite(Green, HIGH);
  CheckDistance();
  obstacleLeft = obstacleDistance;
  Serial.print(obstacleLeft);
  Serial.print("cm left");
  Serial.println();
  digitalWrite(Green, LOW);
  Neck.write(Neckcenter-neckAngle); //turn head right
  chThdSleepMilliseconds(200);
  digitalWrite(Blue, HIGH);
  CheckDistance();
  obstacleRight = obstacleDistance;
  Serial.print(obstacleRight);
  Serial.print("cm right");
  Serial.println();
  digitalWrite(Blue, LOW);
  Neck.write(Neckcenter);
  chMtxLock(&lockMutex);
  if ((obstacleLeft <= obstacleAhead) && (obstacleRight >= obstacleLeft)){
    Obstacle=1;
    }
    if ((obstacleRight <= obstacleAhead) && (obstacleLeft >= obstacleRight)){
      Obstacle=2;
    }
      if (((obstacleLeft <= obstacleAhead && obstacleRight <= obstacleAhead && obstacleCenter <= obstacleAhead) && (obstacleCenter == obstacleLeft && obstacleCenter == obstacleRight)) || (obstacleLeft <= obstacleWarning && obstacleRight <= obstacleWarning && obstacleCenter <= obstacleWarning)){
      Obstacle=3;
  }
      if ((obstacleLeft <= obstacleAlert) || (obstacleRight <= obstacleAlert) || (obstacleCenter <= obstacleAlert)) {
      Obstacle=4;
  }
  chMtxUnlock();
  }
}
void WalkDirection(){
  chMtxLock(&lockMutex);
  int walkToggle = Obstacle;
  chMtxUnlock();
  Serial.print(walkToggle);
  Serial.print(" Case");
  Serial.println();

  switch (walkToggle){
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
    case 4: //obect in Front (both Left and Right detect the object)
      digitalWrite(Red, HIGH);
      Reverse(2,30); //turn around
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
    chThdSleepMilliseconds(100);
    digitalWrite(BuzzerPin, LOW);
}

//Servo Primatives

void TiltRightUp(byte ang, byte sp){
  //tilt right up
  for (int i=0; i<=ang; i+=5){
    Lfoot.write(LFcenter+i);
    Rfoot.write(RFcenter+i);
    chThdSleepMilliseconds(sp);
  }
}
void TiltRightDown(byte ang, byte sp){
  //tilt right down
  for (int i=ang; i>0; i-=5){
    Lfoot.write(LFcenter+i);
    Rfoot.write(RFcenter+i);
    chThdSleepMilliseconds(sp);
  }
}

void TiltLeftUp(byte ang, byte sp){
  //tilt left up
  for (int i=0; i<=ang; i+=5){
    Lfoot.write(LFcenter-i);
    Rfoot.write(RFcenter-i);
    chThdSleepMilliseconds(sp);
  }
}
void TiltLeftDown(byte ang, byte sp){
  //tilt left down
  for (int i=ang; i>0; i-=5){
    Lfoot.write(LFcenter-i);
    Rfoot.write(RFcenter-i);
    chThdSleepMilliseconds(sp);
  }
}

void LeftFootUp(char ang, byte sp){
  //tilt left up
  for (int i=0; i<=ang; i+=5){
    Lfoot.write(LFcenter-i);
    chThdSleepMilliseconds(sp);
  }
}
void LeftFootDown(byte ang, byte sp){
  //tilt left down
  for (int i=ang; i>0; i-=5){
    Lfoot.write(LFcenter-i);
    chThdSleepMilliseconds(sp);
  }
}

void RightFootUp(byte ang, byte sp){
  //tilt right up
  for (int i=0; i<=ang; i+=5){
    Rfoot.write(RFcenter+i);
    chThdSleepMilliseconds(sp);
  }
}
void RightFootDown(byte ang, byte sp){
  //tilt right down
  for (int i=ang; i>0; i-=5){
    Rfoot.write(RFcenter+i);
    chThdSleepMilliseconds(sp);
  }
}

void SwingRight(byte ang, byte sp){
  //swing right
  for (int i=0; i<=ang; i+=5){
    Lleg.write(LLcenter-i);
    Rleg.write(RLcenter-i);
    chThdSleepMilliseconds(sp);
  }
}
void SwingRcenter(byte ang, byte sp){
  //swing r->center
  for (int i=ang; i>0; i-=5){
    Lleg.write(LLcenter-i);
    Rleg.write(RLcenter-i);
    chThdSleepMilliseconds(sp);
  }
}

void SwingLeft(byte ang, byte sp){
  //swing left
  for (byte i=0; i<=ang; i=i+5){
    Lleg.write(LLcenter+i);
    Rleg.write(RLcenter+i);
    chThdSleepMilliseconds(sp);
  }
}
void SwingLcenter(byte ang, byte sp){
  //swing l->center
  for (byte i=ang; i>0; i=i-5){
    Lleg.write(LLcenter+i);
    Rleg.write(RLcenter+i);
    chThdSleepMilliseconds(sp);
  }
}

void RightLegIn(byte ang, byte sp){
  //swing right
  for (int i=0; i<=ang; i+=5){
    Rleg.write(RLcenter-i);
    chThdSleepMilliseconds(sp);
  }
}
void RightLegIcenter(byte ang, byte sp){
  //swing r->center
  for (int i=ang; i>0; i-=5){
    Rleg.write(RLcenter-i);
    chThdSleepMilliseconds(sp);
  }
}

void RightLegOut(byte ang, byte sp){
  //swing right
  for (int i=0; i<=ang; i+=5){
    Rleg.write(RLcenter+i);
    chThdSleepMilliseconds(sp);
  }
}
void RightLegOcenter(byte ang, byte sp){
  //swing r->center
  for (int i=ang; i>0; i-=5){
    Rleg.write(RLcenter+i);
    chThdSleepMilliseconds(sp);
  }
}

void LeftLegIn(byte ang, byte sp){
  //swing left
  for (byte i=0; i<=ang; i=i+5){
    Lleg.write(LLcenter+i);
    chThdSleepMilliseconds(sp);
  }
}
void LeftLegIcenter(byte ang, byte sp){
  //swing l->center
  for (byte i=ang; i>0; i=i-5){
    Lleg.write(LLcenter+i);
    chThdSleepMilliseconds(sp);
  }
}

void LeftLegOut(byte ang, byte sp){
  //swing left
  for (byte i=0; i<=ang; i=i+5){
    Lleg.write(LLcenter-i);
    chThdSleepMilliseconds(sp);
  }
}
void LeftLegOcenter(byte ang, byte sp){
  //swing l->center
  for (byte i=ang; i>0; i=i-5){
    Lleg.write(LLcenter-i);
    chThdSleepMilliseconds(sp);
  }
}

void NeckLeft(byte ang, byte sp){
  //swing left
  for (int i=0; i<=ang; i+=5){
    Neck.write(Neckcenter+i);
    chThdSleepMilliseconds(sp);
  }
}
void NeckRight(byte ang, byte sp){
  //swing right
  for (int i=0; i<=ang; i+=5){
    Neck.write(Neckcenter-i);
    chThdSleepMilliseconds(sp);
  }
}
void NeckIcenter(byte ang, byte sp){
  //swing neck->center
  for (byte i=ang; i>0; i=i-5){
    Neck.write(Neckcenter+i);
    chThdSleepMilliseconds(sp);
  }
}
void NeckOcenter(byte ang, byte sp){
  //swing neck->center
  for (byte i=ang; i>0; i=i-5){
    Neck.write(Neckcenter-i);
    chThdSleepMilliseconds(sp);
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

void Reverse(byte Steps, byte Speed){
  digitalWrite(EnableServo,HIGH);
  TiltLeftUp(tAngle, Speed);
  for (byte j=0; j<Steps; ++j){
    SwingRight(sAngle, Speed);
    TiltLeftDown(tAngle, Speed);
    TiltRightUp(tAngle, Speed);
    SwingRcenter(sAngle, Speed);
    SwingLeft(sAngle, Speed);
    TiltRightDown(tAngle, Speed);
    TiltLeftUp(tAngle, Speed);
    SwingLcenter(sAngle, Speed);
  }
  TiltLeftDown(tAngle, Speed);
  digitalWrite(EnableServo,LOW);
}

void TurnLeft(byte Steps, byte Speed){
  digitalWrite(EnableServo,HIGH);
  TiltLeftUp(uAngle, Speed);
  chThdSleepMilliseconds(20);
  for (byte j=0; j<Steps; ++j){
    LeftLegIn(sAngle, Speed);
    TiltLeftDown(uAngle, Speed);
    TiltRightUp(uAngle, Speed);
    chThdSleepMilliseconds(20);
    LeftLegIcenter(sAngle, Speed);
    RightLegOut(sAngle, Speed);
    TiltRightDown(uAngle, Speed);
    TiltLeftUp(uAngle, Speed);
    chThdSleepMilliseconds(20);
    RightLegOcenter(sAngle, Speed);
  }
  TiltLeftDown(uAngle, Speed);
  digitalWrite(EnableServo,LOW);
}

void TurnRight(byte Stps, byte Speed){
  digitalWrite(EnableServo,HIGH);
  TiltRightUp(uAngle, Speed);
  chThdSleepMilliseconds(20);
  for (byte f=0; f<=Stps; ++f){
    RightLegIn(sAngle, Speed);
    TiltRightDown(uAngle, Speed);
    TiltLeftUp(uAngle, Speed);
    chThdSleepMilliseconds(20);
    RightLegIcenter(sAngle, Speed);
    LeftLegOut(sAngle, Speed);
    TiltLeftDown(uAngle, Speed);
    TiltRightUp(uAngle, Speed);
    chThdSleepMilliseconds(20);
    LeftLegOcenter(sAngle, Speed);
  }
  TiltRightDown(uAngle, Speed);
  digitalWrite(EnableServo,LOW);
}
