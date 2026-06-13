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
constexpr int ADC_BUFFER_SIZE = 100;

// Every value below influences control and can be changed from the GUI.
double setpoint = 16.0;       // cm, ball center
double Kp = 2.2;              // (m/s^2) / m
double Ki = 0.5;              // (m/s^2) / (m s)
double Kd = 1.5;              // (m/s^2) / (m/s)
double servoNeutral = 84.0;   // degrees
double servoTravel = 35.0;    // degrees either side of neutral
double controlDirection = -1.0;
double distanceAlpha = 0.25;  // new-sample weight, 0..1
double speedAlpha = 0.18;     // new-sample weight, 0..1
double integralLimit = 12.0;  // cm s
double maxAcceleration = 1.5; // m/s^2
double servoRate = 8.2;       // degrees per 20 ms
double servoDeadband = 0.01;  // minimum PWM update in degrees
double sensorSampleMs = 0.5;  // continuous ADC sample interval
double ballMassKg = 0.0455;
double gravityMps2 = 9.81;
double rollingFactor = 1.4;   // 1 + I/(m*r^2), solid sphere = 1.4
double frictionForceN = 0.003;
double frictionBlendMps2 = 0.03;
double beamPerServo = 0.114;  // beam angle / servo angle
double calibrationScale = 1.0169491525;
double calibrationOffset = -2.5715254237;
double ballRadiusCm = 0.75;

enum InputSource { SENSOR, VISION };
InputSource inputSource = SENSOR;
bool manualMode = false;
double manualAngle = 84.0;
double visionPosition = -1.0;
unsigned long lastVisionMs = 0;

double rawDistance = -1.0;
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
  adcBuffer[adcIndex] = analogRead(DISTANCE_PIN);
  // if (adcIndex > 0)
  // {
  //   unsigned long diff = adcBuffer[adcIndex] - adcBuffer[adcIndex - 1];
  //   speed
  // }

  
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
    return (millis() - lastVisionMs <= 500) ? visionPosition : -1.0;
  }

  sensorRaw = medianAdc();
  voltage = sensorRaw * 3.3 / 4095.0;
  if (voltage <= 0.10) return -1.0;

  rawDistance = 13.0 / voltage;
  return clampValue(calibrationScale * rawDistance + calibrationOffset + ballRadiusCm, 4.0, 30.0);
}

void resetController()
{
  integral = 0.0;
  speed = 0.0;
  previousPosition = -1.0;
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
  {"calscale", &calibrationScale, 0.1, 3.0},
  {"caloffset", &calibrationOffset, -20.0, 20.0},
  {"ballradius", &ballRadiusCm, 0.0, 5.0},
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
    resetController();
    return;
  }
  if (name == "vision" || name == "ball" || name == "pos") {
    visionPosition = textValue.toFloat();
    lastVisionMs = millis();
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
      return;
    }
  }
}
double avarage10speed = 0;
int avaragecnt = 0;
void updateControl(double dt)
{
  double measured = readPosition();
  if (measured < 0.0) return;

  if (position < 0.0) position = measured;
  position += distanceAlpha * (measured - position);

  if (previousPosition < 0.0) previousPosition = position;
  double measuredSpeed = (position - previousPosition) / dt;
  previousPosition = position;
  speed += speedAlpha * (measuredSpeed - speed);

  error = setpoint - position;
  // integral = clampValue(integral + error * dt, -integralLimit, integralLimit);
  integral = integral + error * dt;
  // PID requests ball acceleration. Position and speed are converted cm -> m.
  pTerm = Kp * (error / 100.0);
  iTerm = Ki * (integral / 100.0);
  dTerm = -Kd * (speed / 100.0);
  desiredAcceleration = clampValue(pTerm + iTerm + dTerm,
                                   -maxAcceleration, maxAcceleration);
  pidOutput = desiredAcceleration;
  avaragecnt++;
  avarage10speed += speed;
  Serial.printf("speed %f & %d & error %f\n", speed, avaragecnt, error);

  

  if (avaragecnt == 10)
  {
    avarage10speed = avarage10speed / 10;
    Serial.printf("avarage %f\n", avarage10speed);
    if (fabs(error) < 0.8 && fabs(avarage10speed) < 0.3)
    {
      // writeServo(servoNeutral);
      Serial.printf("DEADBAND!\n");
      avarage10speed = 0;
      avaragecnt = 0;
      return;
    }
    avarage10speed = 0;
    avaragecnt = 0;
      
  }

  
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
      "Plot,Mode:%d,Src:%d,Set:%.3f,Raw:%d,Volt:%.3f,Dist:%.3f,DistF:%.3f,Speed:%.3f,Err:%.3f,Int:%.3f,Accel:%.4f,Out:%.3f,Servo:%.3f,POut:%.4f,IOut:%.4f,DOut:%.4f,Deriv:%.3f,Kp:%.3f,Ki:%.3f,Kd:%.3f,Neutral:%.2f,Travel:%.2f,Dir:%.0f,DistAlpha:%.3f,SpeedAlpha:%.3f,IntLimit:%.2f,MaxAccel:%.3f,ServoRate:%.3f,ServoDead:%.3f,SampleMs:%.2f,Mass:%.4f,Gravity:%.3f,RollFactor:%.3f,Friction:%.5f,FrictionBlend:%.3f,BeamGain:%.4f,Force:%.5f,CalScale:%.5f,CalOffset:%.3f,BallRadius:%.3f\n",
      manualMode, inputSource == VISION, setpoint, sensorRaw, voltage, rawDistance,
      position, speed, error, integral, desiredAcceleration, desiredBeamAngle,
      servoAngle, pTerm, iTerm, dTerm,
      -speed, Kp, Ki, Kd, servoNeutral, servoTravel, controlDirection,
      distanceAlpha, speedAlpha, integralLimit, maxAcceleration, servoRate,
      servoDeadband, sensorSampleMs, ballMassKg, gravityMps2, rollingFactor,
      frictionForceN, frictionBlendMps2, beamPerServo, requiredForceN,
      calibrationScale, calibrationOffset, ballRadiusCm);
}

void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(5);
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
