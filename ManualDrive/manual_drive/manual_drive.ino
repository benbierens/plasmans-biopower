#include <Servo.h>
Servo servoAf;
Servo servoT;

#define ERRORCODE_LOOP_OVER_TIME 101
#define ERRORCODE_RPM_EMERGENCY_STOP 102

// Configuration:
// Program loop speed: milliseconds per loop
#define targetLoopTimeMilliseconds 100
#define loopTimeTolerance 2
#define debugPrintEveryN 5

// Automatic RPM control:
#define targetRPM 1500.0f
#define RPMEmergencyStopThreshold 2500.0f
#define RPMAutomaticAdjustStep 1
#define targetRPMTolerance 5.0f

// Automatic Air/Fuel ratio control:
#define targetAirFuelRatio 1.1f
#define airFuelValueTolerance 5 // In range of [o2ValueMin - o2ValueMax]
#define airFuelAdjustStep 5 // In the range of [airFuelOpenValue - airFuelClosedValue]

#define potentio A2
#define potentioMin 0
#define potentioMax 1024

#define builtInLed 13
#define rpmMeter A1

#define o2sensor A0
#define o2ValueMin 0
#define o2ValueMax 1024

#define servoAirFuel 10
#define airFuelOpenValue 861
#define airFuelClosedValue 2164

#define servoThrottle 9
#define throttleOpenValue 2268
#define throttleClosedValue 1308

#define NUMBEROFGAPS 16.0f
#define MINMILLISECONDSPERGAP 0.83f

int errorCode = 0;
bool ledState = false;
int angle, potValue = 0;
int o2angle, o2Value = 0;
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
int automaticDelay = 100;
int automaticThrottleValue = 0;
int automaticAirFuelValue = 0;
int debugPrintCounter = 0;
int debugAdjustRPM = 0;
int debugAdjustAFR = 0;

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
  while (i < 10)
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
  pinMode(o2sensor, INPUT);
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
  automaticAirFuelValue = airFuelOpenValue + ((airFuelClosedValue - airFuelOpenValue) / 2);

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

  rpmResult = (60.0f * (gapCount * (1000.0f / millisecondsElapsed))) / NUMBEROFGAPS;
}

void loopDelay()
{
  // the number of milliseconds a loop should take is very likely greater than
  // the time it did take. So, sleep the difference.
  // If the millisecondsElapsed is consistently greater than the target loop duration, enter error state.
  if (millisecondsElapsed > (targetLoopTimeMilliseconds + loopTimeTolerance))
  {
    automaticDelay--;
  }
  else if (millisecondsElapsed < (targetLoopTimeMilliseconds - loopTimeTolerance))
  {
    automaticDelay++;
  }

  if (automaticDelay < 10)
  {
    errorState(ERRORCODE_LOOP_OVER_TIME);
  }

  delay(automaticDelay);
}

void manualThrottleControl()
{
  ledOn();

  potValue = analogRead(potentio);
  angle = map(potValue, potentioMin, potentioMax, throttleClosedValue, throttleOpenValue);
  servoT.writeMicroseconds(angle);
}

int numTooHigh = 0;
int i = 0;

void automaticThrottleControl()
{
  ledOff();
// 15.5 - 16
  calculateRpm();

  
  Serial.print("rpm:");
  Serial.print(rpmResult);

  i++;
  if (i >= 10)
  {
    i = 0;
    if (rpmResult < 2)
    {
      Serial.println("*");
      return;
    }
  }
  else
  {
    Serial.println("*");
    return;
  }

  if (rpmResult > RPMEmergencyStopThreshold)
  {
    Serial.println("-toohigh");
    automaticThrottleValue -= (RPMAutomaticAdjustStep * 60);
    numTooHigh++;
    if (numTooHigh > 5)
    {
      servoT.writeMicroseconds(throttleClosedValue);
      errorState(ERRORCODE_RPM_EMERGENCY_STOP);
    }
  }
  else if (rpmResult > (targetRPM + 500 + targetRPMTolerance))
  {
    Serial.println("-!");
    automaticThrottleValue -= (RPMAutomaticAdjustStep * 2);
    debugAdjustRPM--;
  }
  else if (rpmResult < (targetRPM - 500 - targetRPMTolerance))
  {
    Serial.println("+!");
    automaticThrottleValue += (RPMAutomaticAdjustStep);
    debugAdjustRPM++;
  }
  else if (rpmResult > (targetRPM + targetRPMTolerance))
  {
    Serial.println("-");
    automaticThrottleValue -= RPMAutomaticAdjustStep;
    debugAdjustRPM--;
    numTooHigh = 0;
  }
  else if (rpmResult < (targetRPM - targetRPMTolerance))
  {
    Serial.println("+");
    automaticThrottleValue += RPMAutomaticAdjustStep;
    debugAdjustRPM++;
    numTooHigh = 0;
  }
  else
  {
    numTooHigh = 0;
    Serial.println("-OK");
    return;
  }

  if (automaticThrottleValue < throttleClosedValue) automaticThrottleValue = throttleClosedValue;
  if (automaticThrottleValue > throttleOpenValue) automaticThrottleValue = throttleOpenValue;
  servoT.writeMicroseconds(automaticThrottleValue);
}

// voltages of O2 sensor follow linear function:
// value | volts | lambda
//     0 |  0.0v 
//   102 |  0.5v |  0.58
//   614 |  3.0v |  0.99
//   922 |  4.5v |  1.23
//  1024 |  5.0v
const float c = -629.692f;
const float d = (820.0f / 0.65f);
const float targetAsValue = c + (targetAirFuelRatio * d);

void updateAirFuelRatio()
{
  o2Value = analogRead(o2sensor);
  // this is too fast: need temporal filter and averaging.


  // sensor in error? set 50/50 ratio:
  if (o2Value < 110 || o2Value > 920)
  {
    automaticAirFuelValue = airFuelOpenValue + ((airFuelClosedValue - airFuelOpenValue) / 2);    
  }
  else
  {
    if (o2Value > (targetAsValue + airFuelValueTolerance))
    {
      automaticAirFuelValue += airFuelAdjustStep;
      debugAdjustAFR--;
    }
    else if (o2Value < (targetAsValue - airFuelValueTolerance))
    {
      automaticAirFuelValue -= airFuelAdjustStep;
      debugAdjustAFR++;
    }
    else
    {
      return;
    }
  }

  if (automaticAirFuelValue < airFuelOpenValue) automaticAirFuelValue = airFuelOpenValue;
  if (automaticAirFuelValue > airFuelClosedValue) automaticAirFuelValue = airFuelClosedValue;
  //servoAf.writeMicroseconds(automaticAirFuelValue);
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

  updateAirFuelRatio();
  // for testing:
  //calculateRpm();
  //manualThrottleControl();
  automaticThrottleControl();

  if (debugPrintEveryN > 0)
  {
    debugPrintCounter++;
    if (debugPrintCounter >= debugPrintEveryN)
    {
      debugPrintCounter = 0;

      // Serial.print("o2Value:");
      // Serial.print(o2Value);
      // Serial.print(" - rpmResult:");
      // Serial.print(rpmResult);
      // Serial.print(" - automaticThrottleValue:");
      // Serial.print(automaticThrottleValue);
      // Serial.print(" - automaticAirFuelValue:");
      // Serial.print(automaticAirFuelValue);
      // Serial.print(" - automaticDelay:");
      // Serial.print(automaticDelay);
      // Serial.print(" - debugAdjustRPM:");
      // Serial.print(debugAdjustRPM);
      // Serial.print(" - debugAdjustAFR:");
      // Serial.println(debugAdjustAFR);
    }
  }

  //3600 rpm
}
