no Biped with 3C++

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
1
2
3
4
5
6
7
8
9
10
11
12
13
14
15
16
17
18
19
20
21
22
23
24
25
26
27
28
29
30
31
32
33
34
35
36
37
38
39
40
41
42
43
44
45
46
47
48
49
50
51
52
53
54
55
56
57
58
59
60
61
62
63
64
65
66
67
68
69
70
71
72
73
74
75
76
77
78
79
80
81
82
83
84
85
86
87
88
89
90
91
92
93
94
95
96
97
98
99
100
101
102
103
104
105
106
107
108
109
110
111
112
113
114
115
116
117
118
119
120
121
122
123
124
125
126
127
128
129
130
131
132
133
134
135
136
137
138
139
140
141
142
143
144
145
146
147
148
149
150
151
152
153
154
155
156
157
158
159
160
161
162
163
164
165
166
167
168
169
170
171
172
173
174
175
176
177
178
179
180
181
182
183
184
185
186
187
188
189
190
191
192
193
194
195
196
197
198
199
200
201
202
203
204
205
206
207
208
209
210
211
212
213
214
215
216
217
218
219
220
221
222
223
224
225
226
227
228
229
230
231
232
233
234
235
236
237
238
239
240
241
242
243
244
245
246
247
248
249
250
251
252
253
254
255
256
257
258
259
260
261
262
263
264
265
266
267
268
269
270
271
272
273
274
275
276
277
278
279
280
281
282
283
284
285
286
287
288
289
290
291
292
293
294
295
296
297
298
299
300
301
302
303
304
305
306
307
308
309
310
311
312
313
314
315
316
317
318
319
320
321
322
323
324
325
326
327
328
329
330
331
332
333
334
335
336
337
338
339
340
341
342
343
344
345
346
347
348
349
350
351
352
353
354
355
356
357
358
359
360
361
362
363
364
365
366
367
368
369
370
371
372
373
374
375
376
377
378
379
380
381
382
383
384
385
386
387
388
389
390
391
392
393
394
395
396
397
398
399
400
401
402
403
404
405
406
407
408
409
410
411
412
413
414
415
416
417
418
419
420
421
422
423
424
425
426
427
428
429
430
431
432
433
434
435
436
437
438
439
440
441
442
443
444
445
446
447
448
449
450
451
452
453
454
455
456
457
458
459
460
461
462
463
464
465
466
467
468
469
470
471
472
473
474
475
476
477
478
479
480
481
482
483
484
485
486
487
488
489
490
491
492
493
494
495
496
497
498
499
500
501
502
503
504
505
506
507
508
509
510
511
512
513
514
515
516
517
518
519
520
521
522
523
524
525
526
527
528
529
530
531
532
533
534
535
536
537
538
539
540
541
542
543
544
545
546
547
548
549
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

void TurnRight(byte Steps, byte Speed){
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
