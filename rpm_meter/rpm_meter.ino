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

int angle,potval=0;
int openValue=0;
int closeValue=0;
int actualValue,targetValue=0;
int inertia=16;// change this parrameter. Higher the value, the slower the servo rotation speed
int rpmValue = 0;
unsigned long lastTime = 0;
unsigned long now = 0;
unsigned long millisecondsElapsed = 0;
int rpmLastState = 0;
int rpmResult = 0;
int rpmCount = 0;

void setup()
{
  Serial.begin(115200);
  pinMode(potentio,INPUT);
  // pinMode(o2,INPUT);
  servoAf.attach(servoAirFuel);
  servoT.attach(servoThrottle);

  pinMode(rpmMeter, INPUT);

  delay(3000);

  servoAf.writeMicroseconds(1512); // sets air/fuel to 50%
}

void processTick()
{
  now = millis();
  if (lastTime == 0)
  {
    lastTime = now;
    return;
  }

  millisecondsElapsed = now - lastTime;
  lastTime = now;

  rpmResult = (250 / millisecondsElapsed) * 240) / 16;
  rpmCount++;
  if (rpmCount  > 9)
  {
    rpmCount = 0;
    Serial.print("rpm:");
    Serial.println(rpmResult);
  }
}

void loop()
{
  potval=analogRead(potentio);
  angle=map(potval,0,1024,1308,2268);
  servoT.writeMicroseconds(angle);

  rpmValue = analogRead(rpmMeter);
  if (rpmValue < 200) // gap in detector
  {
    if (rpmLastState == 0)
    {
      rpmLastState = 1;
      processTick();
    }
  } 
  else
  {
    rpmLastState = 0;
  } 
}






myservo.writeMicroseconds(X)

luchtbrandstof: pin 10 - "ser1"
Open:  861
Dicht: 2164


throttle:	pin 9 - "ser2"
open:  2268
dicht: 1308


A1 = input: toerenteller

dicht => 990
gat   =>   0


16x voor 1 ronde
1000 -> 0 -> 1000

max 4000 RPM
66.66667 Rpsecond
1067 signals per second

