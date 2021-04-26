#include <Arduino.h>

// Go Button //
#define goBtn 13
bool go = false;

// Motor Driver //
#define inputL1 9
#define inputL2 10
#define inputR1 11
#define inputR2 8
#define enableL 5
#define enableR 6

// Line Sensor //
#define ls1 A0
#define ls2 A1
#define ls3 A2
#define ls4 A3
#define ls5 A4
String lineColor = "white";
bool L2, L1, M, R1, R2;
bool lc;


// Obst. Sensor //
#define obstSensor 2
bool obstDetected = false;

// Function Declarations //
void goPressed();
void goStraight();
void lineSensor();
void obstacleSensor();
void stop();
void slightLeft();
void hardLeft();
void slightRight();
void hardRight();
void systemCheck();
void lineFollow();

void setup() 
{
  Serial.begin(9600);

  // Go Button //
  pinMode(goBtn, INPUT);

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
  {  lc = 0; }
  else if (lineColor == "white" || lineColor == "White" || lineColor == "WHITE")
  {  lc = 1; }

  // Obst. Sensor //
  pinMode(obstSensor, INPUT);

}

void loop() 
{
  goPressed();
  //systemCheck();

  if(go && !obstDetected)
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
  go = digitalRead(goBtn);
}

void goStraight()
{
  digitalWrite(inputL1, HIGH);
  digitalWrite(inputL2, LOW);
  analogWrite(enableL, 250);

  digitalWrite(inputR1, HIGH);
  digitalWrite(inputR2, LOW);
  analogWrite(enableR, 255);
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
  
  if(obstDetected)
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
}

void slightLeft()
{
  digitalWrite(inputL1, HIGH);
  digitalWrite(inputL2, LOW);
  analogWrite(enableL, 55);

  digitalWrite(inputR1, HIGH);
  digitalWrite(inputR2, LOW);
  analogWrite(enableR, 255);
}

void slightRight()
{
  digitalWrite(inputL1, HIGH);
  digitalWrite(inputL2, LOW);
  analogWrite(enableL, 255);

  digitalWrite(inputR1, HIGH);
  digitalWrite(inputR2, LOW);
  analogWrite(enableR, 150);
}

void hardLeft()
{
  digitalWrite(inputL1, LOW);
  digitalWrite(inputL2, HIGH);
  analogWrite(enableL, 135);
  delay(6);
  digitalWrite(inputL1, LOW);
  digitalWrite(inputL2, HIGH);
  analogWrite(enableL, 45);

  digitalWrite(inputR1, HIGH);
  digitalWrite(inputR2, LOW);
  analogWrite(enableR, 255);
}

void hardRight()
{
  digitalWrite(inputL1, HIGH);
  digitalWrite(inputL2, LOW);
  analogWrite(enableL, 255);

  digitalWrite(inputR1, HIGH);
  digitalWrite(inputR2, LOW);
  analogWrite(enableR, 0);
}
void reverse()
{
  digitalWrite(inputL1, LOW);
  digitalWrite(inputL2, HIGH);
  analogWrite(enableL, 250);

  digitalWrite(inputR1, LOW);
  digitalWrite(inputR2, HIGH);
  analogWrite(enableR, 255);
}

void lineFollow()
{
 if(!go)
 {
   stop();
 } 
  // L2 is on the line //
 else if (L2 == lc)
 {
   reverse();
   delay(30);
   hardLeft();
   Serial.println("hardLeft");
 }
  // R2 is on the line //
 else if (R2 == lc)
 {
   reverse();
   delay(30);
   hardRight();
   Serial.println("hardRight");
 }
  // Only M is on the line //
 else if (M == lc && L1 == !lc && R1 == !lc)
 {
   goStraight();
   Serial.println("goStraight");
 }
  // L1 is on the line & M is not on the line //
 else if (M == !lc && L1 == lc)
 {
   slightLeft();
   Serial.println("slightLeft");
 }
  // R1 is on the line & M is not on the line //
 else if (M == !lc && R1 == lc)
 {
   slightRight();
   Serial.println("slightRight");
 }
 // Nothing is on the line //
 else if (L2 == !lc && L1 == !lc && M == !lc && R1 == !lc && R2 == !lc)
 {
   goStraight();
   Serial.println("goStraight");
 }
 // M & L1 is on the line //
 else if (M == lc && L1 == lc)
 {
  hardLeft();
  Serial.println("hardLeft");
 }
 // M & R1 is on the line //
 else if (M == lc && R1 == lc)
 {
  slightRight();
  Serial.println("slightRight");
 }
 else
 {
   stop();
   Serial.println("stop");
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