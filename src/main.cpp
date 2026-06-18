#include <Arduino.h>
#include <math.h>

// Hardware
constexpr int SERVO_PIN = 27;
constexpr int SERVO_CHANNEL = 0;
constexpr int SERVO_FREQ_HZ = 50;
constexpr int SERVO_RES_BITS = 16;
constexpr int DISTANCE_PIN = 34;
constexpr int DISPLAY_DIN = 23;
constexpr int DISPLAY_CLK = 18;
constexpr int DISPLAY_CS = 5;

constexpr unsigned long CONTROL_US = 20000; // 50 Hz
constexpr unsigned long TELEMETRY_MS = 200;
constexpr int ADC_BUFFER_SIZE = 31; // 15.5 ms window at the default 0.5 ms sample rate

// Every value below influences control and can be changed from the GUI.
double setpoint = 16.0;       // cm, ball center
double Kp = 2;              // (m/s^2) / m
double Ki = 0.35;             // (m/s^2) / (m s)
double Kd = 1.2;              // (m/s^2) / (m/s)
double servoNeutral = 84.0;   // degrees
double servoTravel = 35.0;    // degrees either side of neutral
double controlDirection = -1.0;
double distanceAlpha = 0.35;  // new-sample weight, 0..1
double speedAlpha = 0.25;     // new-sample weight, 0..1
double integralLimit = 35.0;  // cm s
double maxAcceleration = 3.5; // m/s^2
double servoRate = 3.5;       // degrees per 20 ms
double servoDeadband = 0.01;  // minimum PWM update in degrees
double sensorSampleMs = 0.5;  // continuous ADC sample interval
double ballMassKg = 0.0455;
double gravityMps2 = 9.81;
double rollingFactor = 1.4;   // 1 + I/(m*r^2), solid sphere = 1.4
double frictionForceN = 0.0015;
double frictionBlendMps2 = 0.08;
double beamPerServo = 0.114;  // beam angle / servo angle
double sensorCurveA = 12.08;  // GP2Y0A41SK0F: distance = A * voltage^exponent
double sensorCurveExponent = -1.058;
// Beam-scale calibration: sensor 10.0 cm -> beam 10.0 cm,
// sensor 14.5 cm -> beam 16.0 cm.
double calibrationScale = 1.3333333333;
double calibrationOffset = -3.3333333333;
double ballRadiusCm = 0.75;
double settleErrorCm = 0.8;
double settleAverageSpeedCmS = 0.5;

enum InputSource { SENSOR, VISION };
InputSource inputSource = SENSOR;
bool manualMode = false;
double manualAngle = 84.0;
double visionPosition = -1.0;
unsigned long lastVisionMs = 0;
constexpr unsigned long VISION_TIMEOUT_MS = 250;

double rawDistance = -1.0;
double measuredPosition = -1.0;
double position = -1.0;
double previousPosition = -1.0;
double speed = 0.0;
double error = 0.0;
double integral = 0.0;
double pTerm = 0.0;
double iTerm = 0.0;
double dTerm = 0.0;
double pidOutput = 0.0;
double desiredAcceleration = 0.0;
double desiredBeamAngle = 0.0;
double requiredForceN = 0.0;
double avarage10speed = 0.0;
double averageSpeed10 = 0.0;
int avaragecnt = 0;
bool settleActive = false;
double servoAngle = 84.0;
double writtenServoAngle = 84.0;
int sensorRaw = 0;
double voltage = 0.0;
unsigned long lastControlUs = 0;
unsigned long lastTelemetryMs = 0;
unsigned long lastSensorSampleUs = 0;
int adcBuffer[ADC_BUFFER_SIZE];
int adcIndex = 0;
int adcCount = 0;

double clampValue(double value, double low, double high)
{
  return fmin(high, fmax(low, value));
}

void writeServo(double angle)
{
  angle = clampValue(angle, 0.0, 180.0);
  const double pulseUs = 500.0 + angle * (1900.0 / 180.0);
  const uint32_t maxDuty = (1UL << SERVO_RES_BITS) - 1;
  ledcWrite(SERVO_CHANNEL, lround(pulseUs * maxDuty / 20000.0));
}

void displaySend(byte reg, byte data)
{
  digitalWrite(DISPLAY_CS, LOW);
  shiftOut(DISPLAY_DIN, DISPLAY_CLK, MSBFIRST, reg);
  shiftOut(DISPLAY_DIN, DISPLAY_CLK, MSBFIRST, data);
  digitalWrite(DISPLAY_CS, HIGH);
}

void initDisplay()
{
  pinMode(DISPLAY_DIN, OUTPUT);
  pinMode(DISPLAY_CLK, OUTPUT);
  pinMode(DISPLAY_CS, OUTPUT);
  digitalWrite(DISPLAY_CS, HIGH);
  displaySend(0x0F, 0x00);
  displaySend(0x09, 0x0F);
  displaySend(0x0B, 0x03);
  displaySend(0x0A, 0x08);
  displaySend(0x0C, 0x01);
}

void showDistance(double cm)
{
  if (cm < 0.0) {
    for (int digit = 1; digit <= 4; ++digit) displaySend(digit, 0x0A);
    return;
  }

  int value = lround(cm * 10.0);
  displaySend(1, (value % 10) | 0x80);
  value /= 10;
  for (int digit = 2; digit <= 4; ++digit) {
    displaySend(digit, value ? value % 10 : 0x0F);
    value /= 10;
  }
}

void sampleSensor()
{
  if (inputSource != SENSOR) return;

  unsigned long nowUs = micros();
  unsigned long intervalUs = lround(sensorSampleMs * 1000.0);
  if ((unsigned long)(nowUs - lastSensorSampleUs) < intervalUs) return;
  // unsigned long dt = nowUs - lastSensorSampleUs;
  lastSensorSampleUs = nowUs;
  adcBuffer[adcIndex] = analogReadMilliVolts(DISTANCE_PIN);
  adcIndex = (adcIndex + 1) % ADC_BUFFER_SIZE;
  if (adcCount < ADC_BUFFER_SIZE) ++adcCount;
}
static int samples[ADC_BUFFER_SIZE];
int medianAdc()
{
  if (!adcCount) return 0;

  const int count = min(adcCount, ADC_BUFFER_SIZE);
  for (int i = 0; i < count; ++i) samples[i] = adcBuffer[i];
  for (int i = 1; i < count; ++i) {
    int value = samples[i];
    int j = i - 1;
    while (j >= 0 && samples[j] > value) {
      samples[j + 1] = samples[j];
      --j;
    }
    samples[j + 1] = value;
  }
  return samples[count / 2];
}

double readPosition()
{
  if (inputSource == VISION) {
    measuredPosition = (millis() - lastVisionMs <= VISION_TIMEOUT_MS)
        ? visionPosition : -1.0;
    return measuredPosition;
  }

  sensorRaw = medianAdc(); // calibrated millivolts
  voltage = sensorRaw / 1000.0;
  if (voltage <= 0.0) return -1.0;

  rawDistance = sensorCurveA * pow(voltage, sensorCurveExponent);
  if (rawDistance < 4.0 || rawDistance > 30.0) return -1.0;
  double ballCenterDistance = rawDistance + ballRadiusCm;
  measuredPosition = calibrationScale * ballCenterDistance + calibrationOffset;
  return measuredPosition;
}

void resetController()
{
  integral = 0.0;
  speed = 0.0;
  previousPosition = -1.0;
  avarage10speed = 0.0;
  averageSpeed10 = 0.0;
  avaragecnt = 0;
  settleActive = false;
}

void levelServo()
{
  servoAngle += clampValue(servoNeutral - servoAngle, -servoRate, servoRate);
  if (fabs(servoAngle - writtenServoAngle) >= servoDeadband) {
    writtenServoAngle = servoAngle;
    writeServo(servoAngle);
  }
}

struct Parameter {
  const char *name;
  double *value;
  double minimum;
  double maximum;
};

Parameter parameters[] = {
  {"set", &setpoint, 0.0, 30.0},
  {"setpoint", &setpoint, 0.0, 30.0},
  {"kp", &Kp, 0.0, 20.0},
  {"ki", &Ki, 0.0, 10.0},
  {"kd", &Kd, 0.0, 10.0},
  {"neutral", &servoNeutral, 0.0, 180.0},
  {"travel", &servoTravel, 1.0, 60.0},
  {"dir", &controlDirection, -1.0, 1.0},
  {"distalpha", &distanceAlpha, 0.0, 1.0},
  {"speedalpha", &speedAlpha, 0.0, 1.0},
  {"intlimit", &integralLimit, 0.0, 100.0},
  {"maxaccel", &maxAcceleration, 0.01, 10.0},
  {"servorate", &servoRate, 0.01, 20.0},
  {"servodead", &servoDeadband, 0.0, 5.0},
  {"samplems", &sensorSampleMs, 0.25, 20.0},
  {"mass", &ballMassKg, 0.001, 1.0},
  {"gravity", &gravityMps2, 1.0, 20.0},
  {"rollfactor", &rollingFactor, 1.0, 3.0},
  {"friction", &frictionForceN, 0.0, 1.0},
  {"frictionblend", &frictionBlendMps2, 0.001, 1.0},
  {"beamgain", &beamPerServo, 0.001, 1.0},
  {"curvea", &sensorCurveA, 1.0, 30.0},
  {"curveexp", &sensorCurveExponent, -3.0, -0.1},
  {"calscale", &calibrationScale, 0.1, 3.0},
  {"caloffset", &calibrationOffset, -20.0, 20.0},
  {"ballradius", &ballRadiusCm, 0.0, 5.0},
  {"settleerror", &settleErrorCm, 0.0, 5.0},
  {"settlespeed", &settleAverageSpeedCmS, 0.0, 10.0},
};

void handleCommand()
{
  if (!Serial.available()) return;

  String command = Serial.readStringUntil('\n');
  command.trim();
  command.toLowerCase();
  if (!command.length()) return;

  if (command == "auto") {
    manualMode = false;
    resetController();
    return;
  }
  if (command == "reset" || command.startsWith("reset:")) {
    resetController();
    return;
  }

  int separator = command.indexOf(':');
  if (separator < 1) return;
  String name = command.substring(0, separator);
  String textValue = command.substring(separator + 1);

  if (name == "source" || name == "input") {
    inputSource = (textValue == "vision" || textValue == "cv" || textValue == "camera") ? VISION : SENSOR;
    visionPosition = -1.0;
    measuredPosition = -1.0;
    resetController();
    return;
  }
  if (name == "vision" || name == "ball" || name == "pos") {
    if (inputSource != VISION) resetController();
    visionPosition = textValue.toFloat();
    lastVisionMs = millis();
    // A vision sample makes tracking authoritative. The analog sensor remains
    // disabled until an explicit "source:sensor" command is received.
    inputSource = VISION;
    return;
  }
  if (name == "angle" || name == "servo") {
    manualAngle = textValue.toFloat();
    manualMode = true;
    return;
  }
  if (name == "manual") {
    manualMode = textValue == "1" || textValue == "on";
    return;
  }

  for (Parameter &parameter : parameters) {
    if (name == parameter.name) {
      *parameter.value = clampValue(textValue.toFloat(), parameter.minimum, parameter.maximum);
      if (name == "dir") controlDirection = controlDirection >= 0.0 ? 1.0 : -1.0;
      if (name == "set" || name == "setpoint") resetController();
      return;
    }
  }
}
void updateControl(double dt)
{
  double measured = readPosition();
  if (measured < 0.0) {
    if (inputSource == VISION) levelServo();
    return;
  }

  if (position < 0.0) position = measured;
  position += distanceAlpha * (measured - position);

  if (previousPosition < 0.0) previousPosition = position;
  double measuredSpeed = (position - previousPosition) / dt;
  previousPosition = position;
  speed += speedAlpha * (measuredSpeed - speed);

  // Positive error means the ball is too far right/away from the sensor.
  error = position - setpoint;

  // Preserve the original 10-sample average-speed deadband logic.
  avarage10speed += speed;
  ++avaragecnt;
  if (avaragecnt == 10) {
    averageSpeed10 = avarage10speed / 10.0;
    settleActive = fabs(error) < settleErrorCm &&
                   fabs(averageSpeed10) < settleAverageSpeedCmS;
    avarage10speed = 0.0;
    avaragecnt = 0;

    if (settleActive) {
      // Stop applying the previous tilted command while settled.
      levelServo();
      showDistance(position);
      return;
    }
  } else if (settleActive) {
    levelServo();
    showDistance(position);
    return;
  }

  if ((error > 0 && integral < 0) || (error < 0 && integral > 0))
  {
    integral = 0;
  }
  integral = clampValue(integral + error * dt, -integralLimit, integralLimit);

  // Positive acceleration increases sensor distance. Therefore positive error
  // requests negative acceleration, back toward the sensor.
  pTerm = -Kp * (error / 100.0);
  dTerm = -Kd * (speed / 100.0);
  // if (fabs(dTerm) > 0.01)
    // iTerm = 0;
  // else
    iTerm = -Ki * (integral / 100.0);
  desiredAcceleration = clampValue(pTerm + iTerm + dTerm,
                                   -maxAcceleration, maxAcceleration);
  pidOutput = desiredAcceleration;
  // Inverse rolling-ball model:
  // m*g*sin(theta) = rollingFactor*m*a + frictionForce.
  // Smooth friction direction near zero prevents command sign chatter.
  double frictionDirection = desiredAcceleration /
      sqrt(desiredAcceleration * desiredAcceleration +
           frictionBlendMps2 * frictionBlendMps2);
  requiredForceN = rollingFactor * ballMassKg * desiredAcceleration +
                   frictionForceN * frictionDirection;
  double sinBeamAngle = requiredForceN / (ballMassKg * gravityMps2);
  sinBeamAngle = clampValue(sinBeamAngle, -1.0, 1.0);
  desiredBeamAngle = asin(sinBeamAngle) * 180.0 / PI;

  double servoCorrection = desiredBeamAngle / beamPerServo;

  double target = manualMode
      ? manualAngle
      : servoNeutral + controlDirection * servoCorrection;
  target = clampValue(target, servoNeutral - servoTravel,
                      servoNeutral + servoTravel);

  servoAngle += clampValue(target - servoAngle, -servoRate, servoRate);
  if (fabs(servoAngle - writtenServoAngle) >= servoDeadband) {
    writtenServoAngle = servoAngle;
    writeServo(servoAngle);
  }
  showDistance(position);
}

void sendTelemetry()
{
  Serial.printf(
      "Plot,Mode:%d,Src:%d,Set:%.3f,Raw:%d,Volt:%.3f,Dist:%.3f,DistF:%.3f,Speed:%.3f,Avg10Speed:%.3f,Err:%.3f,Int:%.3f,Accel:%.4f,Out:%.3f,Servo:%.3f,POut:%.4f,IOut:%.4f,DOut:%.4f,Deriv:%.3f,Kp:%.3f,Ki:%.3f,Kd:%.3f,Neutral:%.2f,Travel:%.2f,Dir:%.0f,DistAlpha:%.3f,SpeedAlpha:%.3f,IntLimit:%.2f,MaxAccel:%.3f,ServoRate:%.3f,ServoDead:%.3f,SampleMs:%.2f,Mass:%.4f,Gravity:%.3f,RollFactor:%.3f,Friction:%.5f,FrictionBlend:%.3f,BeamGain:%.4f,Force:%.5f,CurveA:%.4f,CurveExp:%.4f,CalScale:%.5f,CalOffset:%.3f,BallRadius:%.3f,SettleError:%.3f,SettleSpeed:%.3f,Settle:%d\n",
      manualMode, inputSource == VISION, setpoint, sensorRaw, voltage, measuredPosition,
      position, speed, averageSpeed10, error, integral, desiredAcceleration, desiredBeamAngle,
      servoAngle, pTerm, iTerm, dTerm,
      -speed, Kp, Ki, Kd, servoNeutral, servoTravel, controlDirection,
      distanceAlpha, speedAlpha, integralLimit, maxAcceleration, servoRate,
      servoDeadband, sensorSampleMs, ballMassKg, gravityMps2, rollingFactor,
      frictionForceN, frictionBlendMps2, beamPerServo, requiredForceN,
      sensorCurveA, sensorCurveExponent, calibrationScale, calibrationOffset,
      ballRadiusCm, settleErrorCm, settleAverageSpeedCmS, settleActive ? 1 : 0);
}

void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(5);
  analogReadResolution(12);
  analogSetPinAttenuation(DISTANCE_PIN, ADC_11db);
  initDisplay();
  ledcSetup(SERVO_CHANNEL, SERVO_FREQ_HZ, SERVO_RES_BITS);
  ledcAttachPin(SERVO_PIN, SERVO_CHANNEL);
  servoAngle = writtenServoAngle = servoNeutral;
  writeServo(servoAngle);
  lastControlUs = micros();
}

void loop()
{
  handleCommand();
  sampleSensor();

  unsigned long nowUs = micros();
  if ((unsigned long)(nowUs - lastControlUs) >= CONTROL_US) {
    double dt = clampValue((nowUs - lastControlUs) / 1000000.0, 0.005, 0.1);
    lastControlUs = nowUs;
    updateControl(dt);
  }

  unsigned long nowMs = millis();
  if (nowMs - lastTelemetryMs >= TELEMETRY_MS) {
    lastTelemetryMs = nowMs;
    sendTelemetry();
  }
}
