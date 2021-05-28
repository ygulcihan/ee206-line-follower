#include <Arduino.h>

// Go Button //
#define goBtn 13
bool go = false;

// Motor Driver //
#define inputL1 5
#define inputL2 4
#define inputR1 2
#define inputR2 7
#define enableL 6
#define enableR 3
unsigned int i = 0;
bool ignore = false;

// Line Sensor //
#define ls1 A4
#define ls2 A3
#define ls3 A2
#define ls4 A1
#define ls5 A0
String lineColor = "white";
bool L2, L1, M, R1, R2;
bool lc;

// Obst. Sensor //
#define obstSensor 0
bool obstDetected = false;

// Reverse Led //
#define rLed 1

// Function Declarations //
void goPressed();
void goStraight(unsigned int speed);
void lineSensor();
void obstacleSensor();
void stop();
void slightLeft(unsigned int speed);
void slightLeftISR();
void hardLeft(unsigned int speed);
void hardLeftISR();
void slightRight(unsigned int speed);
void slightRightISR();
void hardRight(unsigned int speed);
void hardRightISR();
void systemCheck();
void lineFollow();
void reverse();
void setup()
{
  Serial.begin(9600);

  // Go Button //
  pinMode(goBtn, INPUT);
  digitalWrite(goBtn, HIGH);

  // Motor Driver //
  pinMode(inputL1, OUTPUT);
  pinMode(inputL2, OUTPUT);
  pinMode(inputR1, OUTPUT);
  pinMode(inputR2, OUTPUT);
  pinMode(enableL, OUTPUT);
  pinMode(enableR, OUTPUT);

  // Line Sensor //
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);

  if (lineColor == "black" || lineColor == "Black" || lineColor == "BLACK")
  {
    lc = 0;
  }
  else if (lineColor == "white" || lineColor == "White" || lineColor == "WHITE")
  {
    lc = 1;
  }

  // Obst. Sensor //
  pinMode(obstSensor, INPUT);

  // Reverse Led //
  pinMode(rLed, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ls4), slightRightISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ls2), slightLeftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ls1), hardLeftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ls5), hardRightISR, RISING);
}

void loop()
{
  goPressed();
  //systemCheck();

  if (go && !obstDetected)
  {
    lineFollow();
  }

  else
  {
    stop();
  }

  lineSensor();

  obstacleSensor();
}

void goPressed()
{
  go = !digitalRead(goBtn);
}

void goStraight(unsigned int speed)
{
  digitalWrite(inputL1, HIGH);
  digitalWrite(inputL2, LOW);
  analogWrite(enableL, speed);

  digitalWrite(inputR1, HIGH);
  digitalWrite(inputR2, LOW);
  analogWrite(enableR, speed);

  digitalWrite(rLed, LOW);
}

void lineSensor()
{
  L2 = digitalRead(ls5);
  L1 = digitalRead(ls4);
  M = digitalRead(ls3);
  R1 = digitalRead(ls2);
  R2 = digitalRead(ls1);
}

void obstacleSensor()
{
  obstDetected = !digitalRead(obstSensor);

  if (obstDetected)
  {
    stop();
  }
}

void stop()
{
  analogWrite(enableL, 0);
  analogWrite(enableR, 0);
  digitalWrite(inputL1, LOW);
  digitalWrite(inputL2, LOW);
  digitalWrite(inputR1, LOW);
  digitalWrite(inputR2, LOW);

  digitalWrite(rLed, LOW);
}

void slightLeft(unsigned int speed)
{
  digitalWrite(inputL1, HIGH);
  digitalWrite(inputL2, LOW);
  analogWrite(enableL, 80);

  digitalWrite(inputR1, HIGH);
  digitalWrite(inputR2, LOW);
  analogWrite(enableR, speed);
}

void slightLeftISR()
{
  digitalWrite(inputL1, HIGH);
  digitalWrite(inputL2, LOW);
  analogWrite(enableL, 80);

  digitalWrite(inputR1, HIGH);
  digitalWrite(inputR2, LOW);
  analogWrite(enableR, 250);
}

void slightRight(unsigned int speed)
{
  if(ignore)
  {
    ignore = false;
    return;
  }

  else
{
  digitalWrite(inputL1, HIGH);
  digitalWrite(inputL2, LOW);
  analogWrite(enableL, speed);

  digitalWrite(inputR1, HIGH);
  digitalWrite(inputR2, LOW);
  analogWrite(enableR, 85);
}
}

void slightRightISR()
{
  digitalWrite(inputL1, HIGH);
  digitalWrite(inputL2, LOW);
  analogWrite(enableL, 250);

  digitalWrite(inputR1, HIGH);
  digitalWrite(inputR2, LOW);
  analogWrite(enableR, 85);
}

void hardLeft(unsigned int speed)
{
  digitalWrite(inputL1, LOW);
  digitalWrite(inputL2, HIGH);
  analogWrite(enableL, 145);
  delay(6);
  digitalWrite(inputL1, LOW);
  digitalWrite(inputL2, HIGH);
  analogWrite(enableL, 45);

  digitalWrite(inputR1, HIGH);
  digitalWrite(inputR2, LOW);
  analogWrite(enableR, speed);
}

void hardLeftISR()
{
  digitalWrite(inputL1, LOW);
  digitalWrite(inputL2, HIGH);
  analogWrite(enableL, 105);

  digitalWrite(inputR1, HIGH);
  digitalWrite(inputR2, LOW);
  analogWrite(enableR, 160);
}

void hardRight(unsigned int speed)
{
  digitalWrite(inputL1, HIGH);
  digitalWrite(inputL2, LOW);
  analogWrite(enableL, speed);

  digitalWrite(inputR1, LOW);
  digitalWrite(inputR2, HIGH);
  analogWrite(enableR, 145);
  delay(6);
  digitalWrite(inputR1, LOW);
  digitalWrite(inputR2, HIGH);
  analogWrite(enableR, 45);
}

void hardRightISR()
{
  digitalWrite(inputL1, HIGH);
  digitalWrite(inputL2, LOW);
  analogWrite(enableL, 160);

  digitalWrite(inputR1, LOW);
  digitalWrite(inputR2, HIGH);
  analogWrite(enableR, 105);
}
void reverse()
{
  digitalWrite(inputL1, LOW);
  digitalWrite(inputL2, HIGH);
  analogWrite(enableL, 255);

  digitalWrite(inputR1, LOW);
  digitalWrite(inputR2, HIGH);
  analogWrite(enableR, 255);
}

void lineFollow()
{
  if (!go)
  {
    stop();
  }

  else if (M == lc && L1 == lc && L2 == lc)
  {
    ignore = true;
    reverse();
    delay(30);
    hardLeft(240);
    delay(270);
  }
  else if (M == lc && R1 == lc && R2 == lc)
  {
    reverse();
    delay(30);
    hardRight(240);
    delay(400);
  }

  else if (M == lc && L1 == lc)
  {
    hardLeft(140);
  }

  else if (M == lc && R1 == lc)
  {
    hardRight(140);
  }

  // L2 is on the line //
  else if (L2 == lc)
  {
    hardLeft(180);
  }
  // R2 is on the line //
  else if (R2 == lc)
  {
    hardRight(180);
  }

  // L1 is on the line & M is not on the line //
  else if (M == !lc && L1 == lc)
  {
    slightLeft(210);
  }
  // R1 is on the line & M is not on the line //
  else if (M == !lc && R1 == lc)
  {
    slightRight(210);
  }

  // L2 & L1 is on the line //
  else if (L2 == lc && L1 == lc)
  {
    ignore = true;
    reverse();
    delay(30);
    hardLeft(240);
    delay(270);
  }
  // R2 & R1 is on the line //
  else if (R2 == lc && R1 == lc)
  {
    reverse();
    delay(30);
    hardRight(240);
    delay(400);
  }
  // Only M is on the line //
  else if (M == lc && L1 == !lc && R1 == !lc)
  {
    goStraight(175);
  }
  // Nothing is on the line //
  else if (L2 == !lc && L1 == !lc && M == !lc && R1 == !lc && R2 == !lc)
  {
    goStraight(155);
  }
  else
  {
    stop();
  }
}

void systemCheck()
{
  Serial.print("Line Sensor: ");
  Serial.print(L2);
  Serial.print(", ");
  Serial.print(L1);
  Serial.print(", ");
  Serial.print(M);
  Serial.print(", ");
  Serial.print(R1);
  Serial.print(", ");
  Serial.print(R2);

  Serial.print("  obst: ");
  Serial.print(obstDetected);

  Serial.print("  Go: ");
  Serial.print(go);

  Serial.println("");

  delay(100);
}