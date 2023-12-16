#include <Servo.h>
Servo servoAf;
Servo servoT;

#define targetLoopTimeMilliseconds 100

#define potentio A2
#define potentioMin 0
#define potentioMax 1024

#define o2 A0
#define rpmMeter A1

#define servoAirFuel 10
#define airFuelOpenValue 861
#define airFuelClosedValue 2164

#define servoThrottle 9
#define throttleOpenValue 2268
#define throttleClosedValue 1308

#define NUMBEROFGAPS 16.0f
#define MINMILLISECONDSPERGAP 0.83f

int angle,potval=0;
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

  servoAf.attach(servoAirFuel);
  servoT.attach(servoThrottle);

  pinMode(rpmMeter, INPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), onGapSignal, RISING);

  delay(3000);
  rpmCount = 0;

  int airFuelHalf = airFuelClosedValue + ((airFuelOpenValue - airFuelClosedValue) / 2);
  servoAf.writeMicroseconds(airFuelHalf);
  lastNow = millis();
  lastGapMillis = millis();
  now = lastNow;
  gapCounter = 0;
}

void onGapSignal()
{
  gapCounter++;
}

void calculateRpm()
{
  now = millis();
  millisecondsElapsed = now - lastNow;
  gapCount = gapCounter;
  lastNow = now;
  gapCounter = 0;

  rpmResult = (60.0f * (gapCount * (100000.0f / millisecondsElapsed))) / NUMBEROFGAPS;
}

void loop()
{
  potval=analogRead(potentio);
  angle=map(potval,0,1024,1308,2268);
  servoT.writeMicroseconds(angle);

  delay(500);

  calculateRpm();
  Serial.print("rpm:");
  Serial.println(rpmResult / 100);
}


