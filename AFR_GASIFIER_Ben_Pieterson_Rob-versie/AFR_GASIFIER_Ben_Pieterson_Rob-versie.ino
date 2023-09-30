#include <Servo.h>
Servo myservo;
#define potentio A2
#define servo 10
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

void setup()
{
  Serial.begin(115200);
  // pinMode(potentio,INPUT);
  // pinMode(o2,INPUT);
  // myservo.attach(servo);

  pinMode(rpmMeter, INPUT);
}

void loop()
{
  // potval=analogRead(potentio);
  // angle=map(potval,0,1024,600,2400);
  // myservo.writeMicroseconds(angle); // sets the servo position according to the scaled value
  // actualValue=angle;
  // delay(15); // for servo to come to that point
  // Serial.print("POTCONTROL:");
  // Serial.println(angle);
  rpmValue = analogRead(rpmMeter);
  Serial.print("RPM:");
  Serial.println(rpmValue);
  delay(10);
}
