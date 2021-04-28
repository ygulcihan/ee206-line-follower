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
  {  lc = 0; }
  else if (lineColor == "white" || lineColor == "White" || lineColor == "WHITE")
  {  lc = 1; }

  // Obst. Sensor //
  pinMode(obstSensor, INPUT);

  // Reverse Led //
  pinMode(rLed, OUTPUT);

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
  go = !digitalRead(goBtn);
}

void goStraight()
{
  digitalWrite(inputL1, HIGH);
  digitalWrite(inputL2, LOW);
  analogWrite(enableL, 190);

  digitalWrite(inputR1, HIGH);
  digitalWrite(inputR2, LOW);
  analogWrite(enableR, 255);

  digitalWrite(rLed, LOW);

  Serial.println("goStraight");
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

  digitalWrite(rLed, LOW);

  Serial.println("Stop");
}

void slightLeft()
{
  digitalWrite(inputL1, HIGH);
  digitalWrite(inputL2, LOW);
  analogWrite(enableL, 15);

  digitalWrite(inputR1, HIGH);
  digitalWrite(inputR2, LOW);
  analogWrite(enableR, 255);

  digitalWrite(rLed, LOW);

  Serial.println("slightLeft");
}

void slightRight()
{
  digitalWrite(inputL1, HIGH);
  digitalWrite(inputL2, LOW);
  analogWrite(enableL, 255);

  digitalWrite(inputR1, HIGH);
  digitalWrite(inputR2, LOW);
  analogWrite(enableR, 170);

  digitalWrite(rLed, LOW);

  Serial.println("slightRight");
}

void hardLeft()
{
  digitalWrite(inputL1, LOW);
  digitalWrite(inputL2, HIGH);
  analogWrite(enableL, 155);
  delay(6);
  digitalWrite(inputL1, LOW);
  digitalWrite(inputL2, HIGH);
  analogWrite(enableL, 45);

  digitalWrite(inputR1, HIGH);
  digitalWrite(inputR2, LOW);
  analogWrite(enableR, 255);

  digitalWrite(rLed, LOW);


  Serial.println("hardLeft");
}

void hardRight()
{
  digitalWrite(inputL1, HIGH);
  digitalWrite(inputL2, LOW);
  analogWrite(enableL, 255);

  digitalWrite(inputR1, HIGH);
  digitalWrite(inputR2, LOW);
  analogWrite(enableR, 0);

  digitalWrite(rLed, LOW);

  Serial.println("hardRight");
}
void reverse()
{
  digitalWrite(inputL1, LOW);
  digitalWrite(inputL2, HIGH);
  analogWrite(enableL, 190);

  digitalWrite(inputR1, LOW);
  digitalWrite(inputR2, HIGH);
  analogWrite(enableR, 255);

  digitalWrite(rLed, HIGH);
  delay(100);
  digitalWrite(rLed, LOW);

  Serial.println("reverse");
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
   stop();
   delay(10);
   reverse();
   delay(30);
   hardLeft();
   delay(200);
 }
  // R2 is on the line //
 else if (R2 == lc)
 {
   stop();
   delay(10);
   reverse();
   delay(30);
   hardRight();
   delay(200);
 }
  // Only M is on the line //
 else if (M == lc && L1 == !lc && R1 == !lc)
 {
   goStraight();
 }
  // L1 is on the line & M is not on the line //
 else if (M == !lc && L1 == lc)
 {
   slightLeft();
 }
  // R1 is on the line & M is not on the line //
 else if (M == !lc && R1 == lc)
 {
   slightRight();
 }
 // Nothing is on the line //
 else if (L2 == !lc && L1 == !lc && M == !lc && R1 == !lc && R2 == !lc)
 {
   goStraight();
 }
 // M & L1 is on the line //
 else if (M == lc && L1 == lc)
 {
  hardLeft();
 }
 // M & R1 is on the line //
 else if (M == lc && R1 == lc)
 {
  slightRight();
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