#include <Servo.h>

Servo myservo1;
Servo myservo2;

void setup()
 {
myservo1.attach(9);
myservo2.attach(10);
}

void loop() {
  myservo1.write(100);
  myservo2.write(90);

}
