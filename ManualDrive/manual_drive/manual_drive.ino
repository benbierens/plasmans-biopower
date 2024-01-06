#include <Servo.h>
#include <PID_v1.h>
Servo servoAf;
Servo servoT;

#define ERRORCODE_LOOP_OVER_TIME 101
#define ERRORCODE_RPM_EMERGENCY_STOP 102

///////////////////
// Configuration //
///////////////////

// Automatic RPM control:
#define targetRPM 1500.0f
#define RPMEmergencyStop 3000.0f
#define NumberOfGaps 16.0f // Number of gaps per 1 full rotation.
// PID-parameters:
#define pid_Kp 2 // Contribution of immediate error to control output.
#define pid_Ki 3 // Contribution of integral error (history of error over time) to control output.
#define pid_Kd 1 // Contribution of differential error (rate of change of error over time) to the control output.

// Automatic Air/Fuel ratio control:
#define targetAirFuelRatio 1.1f
#define airFuelAdjustStep 5 // Must be 1 or greater. Higher number gives larger response step. In the range of [airFuelOpenValue - airFuelClosedValue]

// Infinite Response Filters
// Values must be 2 or greater. Higher number gives slower, smoother response.
#define rpmIRF_divisor 2
#define airFuelIRF_divisor 2

// Program loop speed: milliseconds per loop
#define targetLoopTimeMilliseconds 100
#define loopTimeTolerance 2

///////////
// Debug //
///////////

// #define _DEBUG_
#ifdef _DEBUG_
#define _DEBUG_N 10
#endif

////////////////////////
// Wiring & Constants //
////////////////////////
#define potThrottle A2
#define potThrottleMin 0
#define potThrottleMax 1024

#define potAirFuel A4
#define potAirFuelMin 0
#define potAirFuelMax 1024

#define potManualAutomatic A3
#define potManualAutomaticMin 0
#define potManualAutomaticMax 1024
#define potManualManualUpperThreshold 400 // Values under this = manual control
#define potManualAutomaticLowerThreshold 624 // Values above this = automatic control

#define builtInLed 13
#define rpmInterruptPin 2

#define o2sensor A0
#define o2ValueMin 0
#define o2ValueMax 1024

#define servoAirFuel 10
#define airFuelOpenValue 861
#define airFuelClosedValue 2164
#define airFuelValueTolerance 5.0f // In range of [o2ValueMin - o2ValueMax]

#define servoThrottle 9
#define throttleOpenValue 2268
#define throttleClosedValue 1308

int errorCode = 0;
bool ledState = false;
int potValue = 0;
float o2Value = 0.0f;
float lastNow = 0.0f;
float now = 0.0f;
float millisecondsElapsed = 0.0f;
int rpmLastState = 0;
float rpmResult = 0.0f;
const byte interruptPin = 2;
volatile unsigned int gapCounter = 0;
float gapCount = 0.0f;
float lastGapMillis = 0.0f;
bool isManual = false;
int automaticLoopDelay = 100;
int currentThrottleValue = 0;
int currentAirFuelValue = 0;
#ifdef _DEBUG_
int debug_n = 0;

void log_debug(String s)
{
  if (debug_n != 0) return;
  Serial.print(s);
  Serial.print(" - ");
}

void log_debug_float(String s, float f)
{
  if (debug_n != 0) return;
  Serial.print(s);
  Serial.print(f);
  Serial.print(" - ");
}

void log_debug_double(String s, double d)
{
  if (debug_n != 0) return;
  Serial.print(s);
  Serial.print(d);
  Serial.print(" - ");
}

void log_debug_line()
{
  if (debug_n != 0) return;
  Serial.println("");
}

#endif

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

bool readIsManual()
{
  return analogRead(potManualAutomatic) < potManualManualUpperThreshold;
}

bool readIsAutomatic()
{
  return analogRead(potManualAutomatic) > potManualAutomaticLowerThreshold;
}

float applyIRF(float currentValue, float newValue, float div)
{
  return
      // Infinite history contribution:
      (currentValue * ((div - 1.0f) / div))
        +
      // Current value contribution:
      (newValue * (1.0f / div));
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

void WaitUntilManual()
{
  while (!isManual)
  {
    delay(300);
    flipLed();

    isManual = readIsManual();
  }

#ifdef _DEBUG_
  log_debug("WaitUntilManual: Finshed!");
  log_debug_line();
#endif
}

double pid_input = 0.0;
double pid_driverOut = 0.0;
double pid_setPoint = targetRPM;
int rpm_emergency_timeout = 0;

PID throttlePid(&pid_input, &pid_driverOut, &pid_setPoint, pid_Kp, pid_Ki, pid_Kd, DIRECT);

void setup()
{
  Serial.begin(115200);
  pinMode(potThrottle, INPUT);
  pinMode(potManualAutomatic, INPUT);
  pinMode(o2sensor, INPUT);
  pinMode(builtInLed, OUTPUT);

  servoAf.attach(servoAirFuel);
  servoT.attach(servoThrottle);

  pinMode(rpmInterruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rpmInterruptPin), onGapSignal, RISING);

  gapCounter = 0;
  throttlePid.SetOutputLimits(throttleClosedValue, throttleOpenValue);
  throttlePid.SetMode(AUTOMATIC); 

  startupDelay();

  currentThrottleValue = throttleClosedValue + ((throttleOpenValue - throttleClosedValue) / 2);
  currentAirFuelValue = airFuelOpenValue + ((airFuelClosedValue - airFuelOpenValue) / 2);
  servoT.writeMicroseconds(currentThrottleValue);
  servoAf.writeMicroseconds(currentAirFuelValue);

  startupDelay();

  WaitUntilManual();

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

  rpmResult = applyIRF(
    rpmResult,
    (60.0f * (gapCount * (1000.0f / millisecondsElapsed))) / NumberOfGaps,
    rpmIRF_divisor);

#ifdef _DEBUG_
  log_debug_float("RPM:", rpmResult);
#endif
}

void loopDelay()
{
  // the number of milliseconds a loop should take is very likely greater than
  // the time it did take. So, sleep the difference.
  // If the millisecondsElapsed is consistently greater than the target loop duration, enter error state.
  if (millisecondsElapsed > (targetLoopTimeMilliseconds + loopTimeTolerance))
  {
    automaticLoopDelay--;
  }
  else if (millisecondsElapsed < (targetLoopTimeMilliseconds - loopTimeTolerance))
  {
    automaticLoopDelay++;
  }

#ifdef _DEBUG_
  if (automaticLoopDelay < 25)
  {
    log_debug("Loop delay < 25");
  }
#endif

  if (automaticLoopDelay < 10)
  {
    errorState(ERRORCODE_LOOP_OVER_TIME);
  }

  delay(automaticLoopDelay);
}

void manualThrottleControl()
{
  potValue = analogRead(potThrottle);
  currentThrottleValue = map(potValue, potThrottleMin, potThrottleMax, throttleClosedValue, throttleOpenValue);
}

void manualAirFuelRatioControl()
{
  potValue = analogRead(potAirFuel);
  currentAirFuelValue = map(potValue, potAirFuelMin, potAirFuelMax, airFuelClosedValue, airFuelOpenValue);
}

void automaticThrottleControl()
{
  if (rpm_emergency_timeout <= 0)
  {
    if (rpmResult > RPMEmergencyStop)
    {
      servoT.writeMicroseconds(throttleClosedValue);
      errorState(ERRORCODE_RPM_EMERGENCY_STOP);
    }
  }
  else
  {
    rpm_emergency_timeout--;
  }

  pid_input = rpmResult;
  if (throttlePid.Compute())
  {
#ifdef _DEBUG_
    log_debug_double("pid_input", pid_input);
    log_debug_double("pid_driverOut", pid_driverOut);
#endif
    currentThrottleValue = pid_driverOut;
  }
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

void calculateO2Value()
{
  o2Value = applyIRF(o2Value, analogRead(o2sensor), airFuelIRF_divisor);
}

void automaticAirFuelRatioControl()
{
  // sensor in error? set 50/50 ratio:
  if (o2Value < 110.0f || o2Value > 920.0f)
  {
#ifdef _DEBUG_
    log_debug("O2-sernsor-offline");
#endif
    currentAirFuelValue = airFuelOpenValue + ((airFuelClosedValue - airFuelOpenValue) / 2);    
  }
  else
  {
#ifdef _DEBUG_
    log_debug_float("o2Value", o2Value);
    log_debug_float("targetAsValue", targetAsValue);
#endif
    if (o2Value > (targetAsValue + airFuelValueTolerance))
    {
      currentAirFuelValue += airFuelAdjustStep;
    }
    else if (o2Value < (targetAsValue - airFuelValueTolerance))
    {
      currentAirFuelValue -= airFuelAdjustStep;
    }
    else
    {
      return;
    }
  }
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

  // Calculated RPM and O2 values are not used in manual mode, but we calculate them anyway, because:
  // - Registers may reach overflow if calculation is not frequently performed.
  // - When switching from manual to automatic, IRFs must be already initialized with accurate values.
  calculateRpm();
  calculateO2Value();

  if (isManual)
  {
  #ifdef _DEBUG_
    log_debug("M");
  #endif
    manualThrottleControl();
    manualAirFuelRatioControl();

    ledOn();
    isManual = !readIsAutomatic();
    if (!isManual)
    {
      // When switching to automatic, we disable rpm_emergency_stop for a short duration.
      rpm_emergency_timeout = (3000 / targetLoopTimeMilliseconds);
      // And we initialize the pid driverOut value with our current throttle value.
      pid_driverOut = currentThrottleValue;
    }
  }
  else
  {
  #ifdef _DEBUG_
    log_debug("A");
  #endif
    automaticThrottleControl();
    automaticAirFuelRatioControl();

    ledOff();
    isManual = readIsManual();
  }

  // Send updates to servos:
  if (currentAirFuelValue < airFuelOpenValue) currentAirFuelValue = airFuelOpenValue;
  if (currentAirFuelValue > airFuelClosedValue) currentAirFuelValue = airFuelClosedValue;
  servoAf.writeMicroseconds(currentAirFuelValue);
  if (currentThrottleValue < throttleClosedValue) currentThrottleValue = throttleClosedValue;
  if (currentThrottleValue > throttleOpenValue) currentThrottleValue = throttleOpenValue;
  servoT.writeMicroseconds(currentThrottleValue);

  #ifdef _DEBUG_
  log_debug_line();
  debug_n++;
  if (debug_n >= _DEBUG_N) debug_n = 0;
  #endif
}
