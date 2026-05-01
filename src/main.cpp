#include <Arduino.h>

// ===== SERVO PWM SETUP (ESP32 LEDC) =====
const int SERVO_PIN = 27;          // GPIO 27 for servo signal
const int SERVO_CHANNEL = 0;       // PWM channel 0
const int SERVO_FREQ_HZ = 50;      // 50 Hz for servo
const int SERVO_RES_BITS = 16;     // 16-bit resolution

// ===== MAX7219 7-SEGMENT DISPLAY =====
const int MAX7219_DIN_PIN = 23;
const int MAX7219_CLK_PIN = 18;
const int MAX7219_CS_PIN = 5;

// ===== SENSOR SELECTION =====
// Uncomment one of the following to select sensor type:
// #define USE_HC_SR04        // HC-SR04 ultrasonic sensor
#define USE_ANALOG_SENSOR  // Original analog distance sensor on GPIO A1

#ifdef USE_HC_SR04
  const int TRIG_PIN = 25;   // GPIO 25 for HC-SR04 trigger
  const int ECHO_PIN = 33;   // GPIO 33 for HC-SR04 echo
#else
  #define dist_sensor 34    // Analog sensor on A1
#endif

bool initServoPwm()
{
  double actualFreq = ledcSetup(SERVO_CHANNEL, SERVO_FREQ_HZ, SERVO_RES_BITS);
  if (actualFreq <= 0) {
    return false;
  }
  ledcAttachPin(SERVO_PIN, SERVO_CHANNEL);
  return true;
}

void writeServoAngle(int angle)
{
  angle = constrain(angle, 0, 180);
  // For 50Hz: full period = 20000us
  // 0° = 500us (2.5%), 180° = 2400us (12%)
  int pulseUs = map(angle, 0, 180, 500, 2400);
  const uint32_t maxDuty = (1UL << SERVO_RES_BITS) - 1;
  uint32_t duty = (uint32_t)((pulseUs * maxDuty) / 20000UL);
  ledcWrite(SERVO_CHANNEL, duty);
}

void max7219Send(byte reg, byte data)
{
  digitalWrite(MAX7219_CS_PIN, LOW);

  for (int bit = 7; bit >= 0; bit--) {
    digitalWrite(MAX7219_CLK_PIN, LOW);
    digitalWrite(MAX7219_DIN_PIN, (reg >> bit) & 0x01);
    digitalWrite(MAX7219_CLK_PIN, HIGH);
  }

  for (int bit = 7; bit >= 0; bit--) {
    digitalWrite(MAX7219_CLK_PIN, LOW);
    digitalWrite(MAX7219_DIN_PIN, (data >> bit) & 0x01);
    digitalWrite(MAX7219_CLK_PIN, HIGH);
  }

  digitalWrite(MAX7219_CS_PIN, HIGH);
}

void initDisplay()
{
  pinMode(MAX7219_DIN_PIN, OUTPUT);
  pinMode(MAX7219_CLK_PIN, OUTPUT);
  pinMode(MAX7219_CS_PIN, OUTPUT);

  digitalWrite(MAX7219_CLK_PIN, HIGH);
  digitalWrite(MAX7219_CS_PIN, HIGH);

  max7219Send(0x0F, 0x00); // display test off
  max7219Send(0x09, 0x0F); // decode mode for all digits
  max7219Send(0x0B, 0x03); // scan digits 0..3
  max7219Send(0x0A, 0x08); // intensity
  max7219Send(0x0C, 0x01); // normal operation

  for (int digit = 1; digit <= 4; digit++) {
    max7219Send(digit, 0x0F); // blank
  }
}

void GP2Y0A41SK0F(double dist)
{
  if (dist < 0) {
    for (int digit = 1; digit <= 4; digit++) {
      max7219Send(digit, 0x0A); // dash
    }
    return;
  }

  int distTenth = (int)(dist * 10.0 + 0.5);
  int whole = distTenth / 10;
  int tenth = distTenth % 10;

  int digits[4] = {0x0F, 0x0F, 0x0F, 0x0F};
  digits[0] = tenth | 0x80; // decimal point after tenths digit

  for (int pos = 1; pos <= 3; pos++) {
    digits[pos] = whole % 10;
    whole /= 10;
    if (whole == 0 && pos < 3) {
      break;
    }
  }

  for (int digit = 1; digit <= 4; digit++) {
    max7219Send(digit, (byte)digits[digit - 1]);
  }
}

int potpin = A0;  // Potentiometer connected to A0

int val;          // Variable to store potentiometer value
unsigned long previousMillis = 0;
const long interval_write_servo = 15;  // Update every 15ms instead of delay(15)
#ifdef USE_HC_SR04
  const long RANGE_READ_INTERVAL_MS = 65; // HC-SR04 needs ~60ms between measurements
  unsigned long lastRangeReadMs = 0;
#endif
double prevError = 0;
double integral = 0;  // Accumulated error for integral term
double Kp = 10.0;
double Ki = 0.01;
double Kd = 2.0;
double distance;
double rawDistance;
double filteredDistance = -1.0;
double setpoint = 5.0;
double output;  // PID controller output
double error;   // Current error for display
int sensorRaw;
float voltage;
bool manualServoOverride = false;
int manualServoAngle = 90;
double filteredServoAngle = 90.0;
double lastPidOutput = 0.0;
double prevControlDistance = -1.0;
double filteredDerivative = 0.0;

const double DISTANCE_FILTER_ALPHA = 0.2;
const double PID_DERIVATIVE_ALPHA = 0.85;
const double PID_OUTPUT_ALPHA = 0.35;
const double PID_ERROR_DEADBAND_CM = 0.10;
const double SERVO_FILTER_ALPHA = 0.25;
const double SERVO_DEADBAND_DEG = 1.0;
const double SERVO_RATE_LIMIT_DEG = 3.0;

// Calibrated from user measurements:
// 5.4 cm printed -> 4.0 cm actual
// 11.3 cm printed -> 10.0 cm actual
const double DISTANCE_CAL_SCALE = 1.0169491525;
const double DISTANCE_CAL_OFFSET = -1.4915254237;

void setup()
{  
  Serial.begin(115200);
  initDisplay();

#ifdef USE_ANALOG_SENSOR
  analogSetPinAttenuation(dist_sensor, ADC_11db);
#endif
  
  // Initialize servo PWM
  if (!initServoPwm()) {
    Serial.println("ERROR: Failed to initialize servo PWM!");
  }
  writeServoAngle(90);  // Center servo
  
  #ifdef USE_HC_SR04
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
  #endif
}

#ifdef USE_HC_SR04
bool measureEchoPulseUs(unsigned long timeoutUs, unsigned long &pulseWidthUs)
{
  unsigned long startWait = micros();
  while (digitalRead(ECHO_PIN) == LOW) {
    if ((unsigned long)(micros() - startWait) > timeoutUs) {
      return false;
    }
  }

  unsigned long pulseStart = micros();
  while (digitalRead(ECHO_PIN) == HIGH) {
    if ((unsigned long)(micros() - pulseStart) > timeoutUs) {
      return false;
    }
  }

  pulseWidthUs = (unsigned long)(micros() - pulseStart);
  return true;
}

double readHC_SR04() {
  // Send 10us trigger pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, LOW);
  
  // Measure pulse duration on echo pin
  unsigned long pulseWidthUs = 0;
  if (measureEchoPulseUs(50000, pulseWidthUs)) {
    // distance(cm) = (time_us * speed_of_sound_cm_per_us) / 2
    double dist = (pulseWidthUs * 0.0343) / 2.0;
    if (dist >= 2.0 && dist <= 400.0) {
      return dist;
    }
  }
  return -1.0;  // No valid echo
}
#endif

void leesSensorEnPot()
{
  #ifdef USE_HC_SR04
    // Read distance from HC-SR04 no faster than sensor can handle
    if (millis() - lastRangeReadMs >= RANGE_READ_INTERVAL_MS) {
      lastRangeReadMs = millis();
      distance = readHC_SR04();
    }
    
    if (distance > 0 && distance <= 30) {
      // Serial.print("Distance: ");
      // Serial.print(distance);
      // Serial.println(" cm (HC-SR04)");
    } else if (distance > 30) {
      // Serial.println("Out of range (HC-SR04)");
    } else {
      // Serial.println("HC-SR04 no echo");
    }
  #else
    // Original analog sensor
    sensorRaw = analogRead(dist_sensor);
    voltage = analogReadMilliVolts(dist_sensor) / 1000.0;

    if (voltage > 0.10) {
      rawDistance = 13.0 / voltage;  // rough inverse fit for GP2Y0A41SK0F
      distance = DISTANCE_CAL_SCALE * rawDistance + DISTANCE_CAL_OFFSET;
      distance = constrain(distance, 4.0, 30.0);
      if (filteredDistance < 0) {
        filteredDistance = distance;
      } else {
        filteredDistance = DISTANCE_FILTER_ALPHA * distance + (1.0 - DISTANCE_FILTER_ALPHA) * filteredDistance;
      }
    } else {
      rawDistance = -1.0;
      distance = -1.0;
      filteredDistance = -1.0;
    }

    GP2Y0A41SK0F(distance);
    if (distance <= 30) {
      // Serial.print("Distance: ");
      // Serial.print(distance);
      // Serial.println(" cm");
    } else {
      Serial.println("Out of range");
    }
  #endif
  
  // setpoint = analogRead(potpin);
  // setpoint = map(setpoint, 0, 1023, 4, 30);
}

double PD_regelaar()
{
  // Bereken fout
  double controlDistance = (filteredDistance >= 0) ? filteredDistance : distance;
  if (controlDistance < 0) {
    return lastPidOutput;
  }

  if (prevControlDistance < 0) {
    prevControlDistance = controlDistance;
  }

  error = setpoint - controlDistance;
  if (fabs(error) < PID_ERROR_DEADBAND_CM) {
    error = 0.0;
  }

  // Differentiate the measured distance instead of the error to avoid setpoint kick.
  double measurementDelta = controlDistance - prevControlDistance;
  prevControlDistance = controlDistance;
  double derivative = -measurementDelta;
  filteredDerivative = PID_DERIVATIVE_ALPHA * filteredDerivative + (1.0 - PID_DERIVATIVE_ALPHA) * derivative;

  integral += error;  // Accumulate error for integral term
  
  // Limit integral windup
  integral = constrain(integral, -1000, 1000);
  // Ki = 0;
  prevError = error;

  // PID uitgang
  double rawOutput = Kp * error + Ki * integral + Kd * filteredDerivative;
  lastPidOutput = PID_OUTPUT_ALPHA * lastPidOutput + (1.0 - PID_OUTPUT_ALPHA) * rawOutput;
  return lastPidOutput;
}

void handleSerialCommand()
{
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    Serial.printf("serial input: %s", cmd);
    if (cmd.length() == 0) return;

    String cmdLower = cmd;
    cmdLower.toLowerCase();

    if (cmdLower == "auto") {
      manualServoOverride = false;
      Serial.println("Servo mode: AUTO");
      return;
    }
    
    // Parse command format: "set:value" or "setpoint:value"
    int colonIdx = cmd.indexOf(':');
    if (colonIdx > 0) {
      String param = cmd.substring(0, colonIdx);
      String valueStr = cmd.substring(colonIdx + 1);
      
      param.toLowerCase();
      String valueLower = valueStr;
      valueLower.toLowerCase();
      
      if (param == "set" || param == "setpoint") {
        double newSetpoint = valueStr.toFloat();
        setpoint = newSetpoint;
        Serial.printf("Setpoint updated to: %.2f\n", setpoint);
      } 
      else if (param == "kp") {
        double newKp = valueStr.toFloat();
        Kp = newKp;
        Serial.printf("Kp updated to: %.2f\n", Kp);
      }
      else if (param == "kd") {
        double newKd = valueStr.toFloat();
        Kd = newKd;
        Serial.printf("Kd updated to: %.2f\n", Kd);
      }
      else if (param == "ki") {
        double newKi = valueStr.toFloat();
        Ki = newKi;
        Serial.printf("Ki updated to: %.3f\n", Ki);
      }
      else if (param == "reset") {
        integral = 0;
        prevError = 0;
        prevControlDistance = -1.0;
        filteredDerivative = 0.0;
        lastPidOutput = 0.0;
        Serial.println("PID state reset");
      }
      else if (param == "angle" || param == "servo") {
        manualServoAngle = constrain((int)round(valueStr.toFloat()), 0, 180);
        manualServoOverride = true;
        Serial.printf("Servo mode: MANUAL | Angle: %d\n", manualServoAngle);
      }
      else if (param == "manual") {
        if (valueLower == "1" || valueLower == "on") {
          manualServoOverride = true;
          Serial.printf("Servo mode: MANUAL | Angle: %d\n", manualServoAngle);
        } else if (valueLower == "0" || valueLower == "off") {
          manualServoOverride = false;
          Serial.println("Servo mode: AUTO");
        }
      }
    }
  }
}

void loop()
{
  unsigned long currentMillis = millis();

  handleSerialCommand();

  leesSensorEnPot();
  // displayDistance(distance);

  output = constrain(PD_regelaar(), -90, 90); // Beperk beweging

  // Zet om naar servo positie (0-180 graden)
  double autoServoAngle = 90.0 - output;
  autoServoAngle = constrain(autoServoAngle, 0.0, 180.0);

  if (!manualServoOverride) {
    double delta = autoServoAngle - filteredServoAngle;
    delta = constrain(delta, -SERVO_RATE_LIMIT_DEG, SERVO_RATE_LIMIT_DEG);
    filteredServoAngle += delta;
    if (fabs(filteredServoAngle - autoServoAngle) < SERVO_DEADBAND_DEG) {
      filteredServoAngle = autoServoAngle;
    }
  } else {
    filteredServoAngle = manualServoAngle;
  }

  int servoAngle = (int)round(filteredServoAngle);

  if (currentMillis - previousMillis >= interval_write_servo) {
    previousMillis = currentMillis;
    writeServoAngle(servoAngle);
    
    // Plot-friendly line: values stay numeric and label each signal.
    Serial.printf("Plot,Mode:%d,Set:%.2f,Raw:%d,Volt:%.3f,Dist:%.2f,DistF:%.2f,Err:%.2f,Int:%.2f,Out:%.2f,Servo:%.2f,Kp:%.2f,Ki:%.3f,Kd:%.2f\n",
                  manualServoOverride ? 1 : 0, setpoint, sensorRaw, voltage, distance, filteredDistance, error, integral, output, filteredServoAngle, Kp, Ki, Kd);
  }
}
