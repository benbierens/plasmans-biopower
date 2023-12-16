#include <Servo.h>
Servo servoAf;
Servo servoT;

#define ERRORCODE_LOOP_OVER_TIME 101
#define ERRORCODE_RPM_EMERGENCY_STOP 102

#define targetLoopTimeMilliseconds 100
#define targetRPM 800.0f
#define RPMEmergencyStopThreshold 1500.0f
#define automaticAdjustStep 1
#define targetRPMTolerance 5.0f

#define potentio A2
#define potentioMin 0
#define potentioMax 1024

#define builtInLed 13
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

int errorCode = 0;
int loopOvertimeCounter = 0;
bool ledState = false;
int angle, potval = 0;
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
int automaticThrottleValue = 0;

void flipLed()
{
  ledState = !ledState;
  digitalWrite(LED_BUILTIN, ledState);
}

void ledOn()
{
  ledState = true;
  digitalWrite(LED_BUILTIN, ledState);
}

void ledOff()
{
  ledState = false;
  digitalWrite(LED_BUILTIN, ledState);
}

void errorState(int error)
{
  errorCode = error;
  while (true)
  {
    Serial.print("errorCode:");
    Serial.println(errorCode);
    delay(500);
    flipLed();
  }
}

void startupDelay()
{
  int i = 0;
  while (i < 5)
  {
    delay(300);
    i++;
    flipLed();
  }
}

void setup()
{
  Serial.begin(115200);
  pinMode(potentio, INPUT);
  pinMode(builtInLed, OUTPUT);

  servoAf.attach(servoAirFuel);
  servoT.attach(servoThrottle);

  pinMode(rpmMeter, INPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), onGapSignal, RISING);

  rpmCount = 0;
  gapCounter = 0;

  startupDelay();

  int airFuelHalf = airFuelClosedValue + ((airFuelOpenValue - airFuelClosedValue) / 2);
  servoAf.writeMicroseconds(airFuelHalf);

  automaticThrottleValue = throttleClosedValue + ((throttleOpenValue - throttleClosedValue) / 2);

  startupDelay();
  lastNow = millis();
  lastGapMillis = millis();
  now = lastNow;
}

void onGapSignal()
{
  gapCounter++;
}

void calculateRpm()
{
  gapCount = gapCounter;
  gapCounter = 0;

  rpmResult = (60.0f * (gapCount * (100000.0f / millisecondsElapsed))) / NUMBEROFGAPS;
}

void loopDelay()
{
  // the number of milliseconds a loop should take is very likely greater than
  // the time it did take. So, sleep the difference.
  // If the millisecondsElapsed is consistently greater than the target loop duration, enter error state.
  if (millisecondsElapsed > targetLoopTimeMilliseconds)
  {
    loopOvertimeCounter++;
    if (loopOvertimeCounter > 5)
    {
      errorState(ERRORCODE_LOOP_OVER_TIME);
    }
  }
  loopOvertimeCounter = 0;

  delay(targetLoopTimeMilliseconds - millisecondsElapsed);
}

void manualThrottleControl()
{
  ledOn();

  potval = analogRead(potentio);
  angle = map(potval, potentioMin, potentioMax, throttleClosedValue, throttleOpenValue);
  servoT.writeMicroseconds(angle);
}

void automaticThrottleControl()
{
  ledOff();

  calculateRpm();

  if (rpmResult > RPMEmergencyStopThreshold)
  {
    servoT.writeMicroseconds(throttleClosedValue);
    errorState(ERRORCODE_RPM_EMERGENCY_STOP);
  }
  else if (rpmResult > (targetRPM + targetRPMTolerance))
  {
    automaticThrottleValue -= automaticAdjustStep;
    if (automaticThrottleValue < throttleClosedValue) automaticThrottleValue = throttleClosedValue;
  }
  else if (rpmResult < (targetRPM - targetRPMTolerance))
  {
    automaticThrottleValue += automaticAdjustStep;
    if (automaticThrottleValue > throttleOpenValue) automaticThrottleValue = throttleOpenValue;
  }
  else
  {
    return;
  }

  servoT.writeMicroseconds(automaticThrottleValue);
}

void loop()
{
  now = millis();
  millisecondsElapsed = now - lastNow;
  lastNow = now;
  if (millisecondsElapsed < 0.0f)
  {
    // millis must have overflowed. We can't use this cycle's time calculation.
    // Apply a short sleep and start over.
    delay(100);
    return;
  }
  loopDelay();

  manualThrottleControl();


}


