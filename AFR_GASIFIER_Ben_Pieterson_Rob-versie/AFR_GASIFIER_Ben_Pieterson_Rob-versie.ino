#include <Servo.h>
Servo myservo;
#define potentio A2//A2
#define servo 10
#define o2 A0//A0

#define OPENED 1670 //was 870
#define CLOSED 870 //was 1670
#define HOLD_MIN 1080
#define HOLD_MAX 1220

int angle,potval=0;
int openValue=0;
int closeValue=0;
int actualValue,targetValue=0;
int inertia=16;// change this parrameter. Higher the value, the slower the servo rotation speed


void setup() {
Serial.begin(115200);
pinMode(potentio,INPUT);
pinMode(o2,INPUT);
myservo.attach(servo);

}

void loop() {
while(millis()<45000)//waits for 32 seconds for the labdasensor to heat up, control by potentio (was 32000)
{
potval=analogRead(potentio);
angle=map(potval,0,1024,600,2400);
myservo.writeMicroseconds(angle); // sets the servo position according to the scaled value
actualValue=angle;
delay(15); // for servo to come to that point
Serial.print("POTCONTROL:");
Serial.println(angle);
}
pd(); //function for controlling valve using o2 sensor
}
//operating range of motor applied to valve is 870uSec (open) to 1670usec (closed)
//o2 significative range is 2.34V-3.12V(38%-47%of the valve)
int readO2Sensor()
{
int o2value=analogRead(o2);
return o2value;
}

int giveMicros(int percent)
{
int milly=map(percent,0,100,OPENED,CLOSED); // in this line try to swap the position of open and closed DO SOME TESTING

return milly;
}

int activeRange(int readings)
{
int volts=map(readings,477,640,38,47); // 38 and 47 are the % of opening of the valve, CAN BE ADJUSTED 
return volts;
}

void pd(){

int oxigen=readO2Sensor();
if ((oxigen>447)&&(oxigen<640)) // if the read o2 value is between the range, actuate the valve
{
targetValue=giveMicros(activeRange(oxigen));
if (actualValue>targetValue)
{
actualValue=actualValue-1;
delay(inertia/2);
}
else
{
actualValue=actualValue+1;
delay(inertia/2);
}

}
if(oxigen<477) //if the read o2 value is going below the lowest treshold, open the Valve
{
actualValue--;
if(actualValue<OPENED) actualValue=OPENED;
delay(inertia);


}

if (oxigen>640) //if the read o2 value is going below the high treshold, close the Valve
{
actualValue++;
if(actualValue>CLOSED) actualValue=CLOSED;
delay(inertia);

}

myservo.writeMicroseconds(actualValue);
}