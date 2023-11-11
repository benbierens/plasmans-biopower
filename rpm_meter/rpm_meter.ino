#include <Servo.h>
Servo servoAf;
Servo servoT;
#define potentio A2
#define servoAirFuel 10
#define servoThrottle 9
#define o2 A0
#define rpmMeter A1

#define OPENED 1670 //was 870
#define CLOSED 870 //was 1670
#define HOLD_MIN 1080
#define HOLD_MAX 1220
#define NUMBEROFGAPS 16.0f
#define MINMILLISECONDSPERGAP 0.83f

int angle,potval=0;
int openValue=0;
int closeValue=0;
int actualValue,targetValue=0;
int inertia=16;// change this parrameter. Higher the value, the slower the servo rotation speed
int rpmValue = 0;
float lastNow = 0.0f;
float now = 0.0f;
float millisecondsElapsed = 0.0f;
int rpmLastState = 0;
float rpmResult = 0.0f;
int rpmCount = 0;
const byte interruptPin = 2;
volatile unsigned int gapCounter = 0;
float gapCount = 0.0f;
float lastGapMillis = 0.0f;

void setup()
{
  Serial.begin(115200);
  pinMode(potentio,INPUT);
  // pinMode(o2,INPUT);
  servoAf.attach(servoAirFuel);
  servoT.attach(servoThrottle);

  pinMode(rpmMeter, INPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), onGapSignal, RISING); // 198 - 196

  delay(3000);
  rpmCount = 0;

  servoAf.writeMicroseconds(1512); // sets air/fuel to 50%
  lastNow = millis();
  lastGapMillis = millis();
  now = lastNow;
  gapCounter = 0;
}

void onGapSignal()
{
  //float n = millis();
  //if ((n - lastGapMillis) < MINMILLISECONDSPERGAP) return;7

  gapCounter++;
  //lastGapMillis = n;
}

void loop()
{
  potval=analogRead(potentio);
  angle=map(potval,0,1024,1308,2268);
  servoT.writeMicroseconds(angle);

  delay(1000);

  now = millis();
  millisecondsElapsed = now - lastNow;
  gapCount = gapCounter;
  lastNow = now;
  gapCounter = 0;

  rpmResult = (60.0f * (gapCount * (100000.0f / millisecondsElapsed))) / NUMBEROFGAPS;

  //rpmCount++;
  //if (rpmCount > 0)
  //{
    //rpmCount = 0;
    Serial.print("d-gaps:");
    Serial.print(gapCount);
    Serial.print("-ms:");
    Serial.print(millisecondsElapsed);
    Serial.print("-rpm:");
    Serial.println(rpmResult / 100);
  //}
} //189 - 192



