#include <Arduino.h>
#include <math.h>

// Ensure M_PI is defined (for asin/atan math)
#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif

// ===== SERVO PWM SETUP (ESP32 LEDC) =====
const int SERVO_PIN = 27;          // GPIO 27 for servo signal
const int SERVO_CHANNEL = 0;       // PWM channel 0
const int SERVO_FREQ_HZ = 50;      // 50 Hz for servo
const int SERVO_RES_BITS = 16;     // 16-bit resolution

// ===== MAX7219 7-SEGMENT DISPLAY =====
const int MAX7219_DIN_PIN = 23;
const int MAX7219_CLK_PIN = 18;
const int MAX7219_CS_PIN = 5;

#define DEBUG 0

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

// ===== SYSTEM PARAMETERS (Knikkerban Ball-on-Beam) =====
// Physical system constants from theoretical model (Lagrangian dynamics)
const double BALL_MASS_KG = 0.0055;         // kg (mass)
const double BALL_DIAMETER_MM = 15.0;       // mm (ball diameter, from physical measurement)
const double BALL_RADIUS_MM = BALL_DIAMETER_MM / 2.0;  // mm (radius = 7.5mm)
const double BALL_RADIUS_M = BALL_RADIUS_MM / 1000.0;  // m (= 0.0075 m)
const double BEAM_L_FRONT_CM = 14.0;        // Distance from pivot to ball (front), cm
const double BEAM_L_BACK_CM = 8.5;          // Distance from pivot to ball (back), cm
const double LEVER_OFFSET_MM = 16.0;        // Lever arm offset, mm
const double BEAM_LENGTH_CM = 14.0;         // Effective beam length for kinematics

// Physics constants
const double GRAVITY_M_S2 = 9.81;           // m/s² (gravitational acceleration)
const double BALL_MOMENT_INERTIA = (2.0/5.0) * BALL_MASS_KG * BALL_RADIUS_M * BALL_RADIUS_M;  // kg·m² (solid sphere: J=2/5*m*r²)
const double INERTIA_FACTOR = 1.0 + (BALL_MOMENT_INERTIA / (BALL_MASS_KG * BALL_RADIUS_M * BALL_RADIUS_M));  // ≈ 1.4 for sphere

// Kinematic/Dynamic relationships:
// Servo angle (theta) → Beam angle (alpha): alpha [rad] = (d/L) * theta [rad]
// Beam angle → Ball acceleration (rolling no-slip constraint from Lagrangian):
//   a_ball = g * sin(alpha) / I_factor
//   where I_factor = 1 + J/(m*R²) ≈ 1.4 accounts for rolling inertia
//
// Control strategy (physics-based):
// 1. IR sensor measures ball position on beam
// 2. PID computes desired ball acceleration from position error
// 3. Convert acceleration to required beam angle: alpha = arcsin(a / (g / I_factor))
// 4. Convert beam angle to servo angle using lever gain
// 5. Servo applies torque → lever tilts beam → gravity accelerates rolling ball

const double LEVER_TO_BEAM_GAIN = LEVER_OFFSET_MM / (BEAM_LENGTH_CM * 10.0);  // ~0.114 rad/rad
double desiredBeamAngleDeg = 0.0;          // Intermediate: desired beam tilt in degrees
double desiredBeamAngleRad = 0.0;          // Intermediate: desired beam tilt in radians
double desiredAccelerationMPS2 = 0.0;      // Intermediate: desired ball acceleration (m/s²)
double ballVelocityMPS = 0.0;              // Estimated ball velocity (m/s)
double ballPositionM = 0.0;                // Ball position in meters (from sensor in cm)

// PID component diagnostics (telemetry)
double pid_p = 0.0;
double pid_i = 0.0;
double pid_d = 0.0;
double pid_deriv_cm_s = 0.0;
double measuredSpeed_cm_s = 0.0;

double SERVO_NEUTRAL_DEG = 84.0;
double SERVO_TRAVEL_LIMIT_DEG = 30.0;
double CONTROL_DIRECTION = -1.0;  // Flip this if the ball moves the wrong way

double servoMinDeg()
{
  return SERVO_NEUTRAL_DEG - SERVO_TRAVEL_LIMIT_DEG;
}

double servoMaxDeg()
{
  return SERVO_NEUTRAL_DEG + SERVO_TRAVEL_LIMIT_DEG;
}

double maxBeamTiltDeg()
{
  return SERVO_TRAVEL_LIMIT_DEG * LEVER_TO_BEAM_GAIN;
}

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
const long interval_write_servo = 30;  // Update every 30ms to reduce servo chatter
#ifdef USE_HC_SR04
  const long RANGE_READ_INTERVAL_MS = 65; // HC-SR04 needs ~60ms between measurements
  unsigned long lastRangeReadMs = 0;
#endif
double prevError = 0;
double integral = 0;  // Accumulated error integral in meter*seconds

// ===== PID TUNING NOTES =====
// System: lightweight ball (5.5g) rolling on tilted beam via servo-driven lever
// - Kp: proportional gain. Units: (m/s²) / m of position error
//   Maps position error (m) → desired acceleration (m/s²)
//   E.g., 1 meter error → Kp m/s² acceleration command
//   Try: Kp = 5-15 for this mass. Start with Kp=10 (10 m/s² per meter of error)
// - Ki: integral gain (cumulative error → correction). Keep small (0.01-0.05)
// - Kd: derivative gain (dampens oscillation). 0.5-2.0 typical for smooth response
// 
// Tuning tips:
// - If ball oscillates: increase Kd or reduce Kp
// - If response is sluggish: increase Kp or Ki
// - Physics naturally filters: gravity provides restoring force on tilted beam
double Kp = 8.0;    // (m/s²) per meter of position error — reduced to avoid overshoot
double Ki = 0.001;  // Small integral, avoid windup
double Kd = 12.0;    // Strong derivative to damp oscillation
double distance;
double rawDistance;
double filteredDistance = -1.0;
double setpoint = 16.0;  // cm from sensor to ball CENTER (mid-beam is a good starting point)
double output;  // PID controller output
double error;   // Current error for display
int sensorRaw;
float voltage;
bool manualServoOverride = false;
int manualServoAngle = 84;
double filteredServoAngle = SERVO_NEUTRAL_DEG;
bool filteredServoAngleInitialized = false;
double lastPidOutput = 0.0;
double prevControlDistance = -1.0;
double filteredDerivative = 0.0;

enum MeasurementSource {
  SOURCE_SENSOR = 0,
  SOURCE_VISION = 1
};

MeasurementSource measurementSource = SOURCE_SENSOR;
double visionDistance = -1.0;
double visionFilteredDistance = -1.0;
unsigned long lastVisionUpdateMs = 0;
const double VISION_FILTER_ALPHA = 0.20;

// ===== timing =====
const unsigned long ADC_INTERVAL_US = 500;   // 2kHz
unsigned long lastAdcUs = 0;

// ===== sensor filtering =====
const int MEDIAN_SIZE = 9;
uint16_t adcBuffer[MEDIAN_SIZE];
int adcIndex = 0;
bool adcFilled = false;

const double DISTANCE_FILTER_ALPHA = 0.10;

// ===== moving average + spike rejection =====
const int MOVING_AVG_SIZE = 8; // simple moving average window
double maBuffer[MOVING_AVG_SIZE];
int maIndex = 0;
bool maFilled = false;
double maSum = 0.0;
double lastValidDistance = -1.0;
const double SPIKE_DISTANCE_THRESHOLD = 3.0; // cm, ignore sudden jumps larger than this

double PID_DERIVATIVE_ALPHA = 0.7;  // Stronger filter to reduce derivative spikes
double PID_OUTPUT_ALPHA = 0.5;      // Moderate output smoothing

double PID_ERROR_DEADBAND_CM = 0.3;

double SERVO_FILTER_ALPHA = 0.15;   // Slightly more responsive filter
double SERVO_DEADBAND_DEG = 0.8;    // Tighter deadband for more precise control
double SERVO_RATE_LIMIT_DEG = 0.5;  // Smaller per-loop step to reduce high-frequency twitching
double BEAM_BREAKAWAY_DEG = 0.35;   // Minimum beam tilt needed to overcome static friction
const unsigned long BREAKAWAY_HOLD_MS = 800;
const unsigned long BREAKAWAY_RAMP_MS = 1800;
// Settle deadband: only zero the output when BOTH error AND derivative are truly tiny.
// Keep these small — a 0.8cm settle window was freezing the servo with 1cm of error.
double SETTLE_ERROR_DEADBAND_CM = 1;   // settle in a small window near setpoint
double SETTLE_DERIVATIVE_DEADBAND = 0.008; // cm/s threshold for "nearly stopped"
const int SERVO_WRITE_MIN_STEP_DEG = 0;
// Calibrated from user measurements:
// 5.4 cm printed -> 4.0 cm actual
// 11.3 cm printed -> 10.0 cm actual
const double DISTANCE_CAL_SCALE = 1.0169491525;
const double DISTANCE_CAL_OFFSET = -1.4915254237;

unsigned long loopCounter = 0;
unsigned long loopTimer = 0;
unsigned long savedCount = 0;
unsigned long adcCount = 0;
unsigned long controlCount = 0;
unsigned long savedAdcCount = 0;
unsigned long savedControlCount = 0;
unsigned long telemetryIntervalMs = 200;
unsigned long lastTelemetryMs = 0;
int lastWrittenServoAngle = (int)SERVO_NEUTRAL_DEG;
unsigned long ballStillSinceMs = 0;

void updateExternalVisionDistance(double dist)
{
  visionDistance = dist;
  lastVisionUpdateMs = millis();

  if (dist < 0) {
    visionFilteredDistance = -1.0;
    distance = -1.0;
    filteredDistance = -1.0;
    return;
  }

  distance = dist;
  if (visionFilteredDistance < 0) {
    visionFilteredDistance = dist;
  } else {
    visionFilteredDistance = VISION_FILTER_ALPHA * dist + (1.0 - VISION_FILTER_ALPHA) * visionFilteredDistance;
  }
  filteredDistance = visionFilteredDistance;
}

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
  writeServoAngle((int)round(SERVO_NEUTRAL_DEG));  // Beam neutral servo position
  filteredServoAngle = SERVO_NEUTRAL_DEG;
  filteredServoAngleInitialized = true;
  
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
uint16_t median9(uint16_t* arr)
{
  uint16_t temp[MEDIAN_SIZE];

  for (int i = 0; i < MEDIAN_SIZE; i++)
    temp[i] = arr[i];

  for (int i = 0; i < MEDIAN_SIZE - 1; i++)
  {
    for (int j = i + 1; j < MEDIAN_SIZE; j++)
    {
      if (temp[j] < temp[i])
      {
        uint16_t t = temp[i];
        temp[i] = temp[j];
        temp[j] = t;
      }
    }
  }

  return temp[MEDIAN_SIZE / 2];
}
void leesSensorEnPot()
{
  if (measurementSource == SOURCE_VISION) {
    if (visionDistance >= 0 && (millis() - lastVisionUpdateMs) <= 500) {
      updateExternalVisionDistance(visionDistance);
    } else {
      updateExternalVisionDistance(-1.0);
    }

    GP2Y0A41SK0F(distance);
    return;
  }

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
    // sample ADC at a fixed rate using ADC_INTERVAL_US to make dt predictable
    if (micros() - lastAdcUs >= ADC_INTERVAL_US)
    {
      lastAdcUs = micros();

      adcBuffer[adcIndex] =
          analogRead(dist_sensor);
      adcCount++;

      adcIndex++;
      if (adcIndex >= MEDIAN_SIZE)
      {
        adcIndex = 0;
        adcFilled = true;
      }
    }

    // only update measurement once enough samples
    if (!adcFilled)
      return;

    // median kills spikes
    sensorRaw =
        median9(adcBuffer);

    // use ESP32 calibrated ADC
    voltage =
        sensorRaw * 3.3 / 4095.0;

    if (voltage > 0.10)
    {
      rawDistance = 13.0 / voltage;

      distance = DISTANCE_CAL_SCALE * rawDistance + DISTANCE_CAL_OFFSET;

      // *** BALL CENTER CORRECTION ***
      // The IR sensor measures distance to the FRONT (nearest surface) of the ball.
      // The ball center is further away by one ball radius (7.5mm = 0.75cm).
      // All setpoints and control logic now refer to ball CENTER position.
      distance += BALL_RADIUS_MM / 10.0;  // convert mm to cm and add

      distance = constrain(distance, 4.0, 30.0);

      // Update simple moving average buffer
      if (maFilled) {
        maSum -= maBuffer[maIndex];
      }
      maBuffer[maIndex] = distance;
      maSum += distance;
      maIndex++;
      if (maIndex >= MOVING_AVG_SIZE) {
        maIndex = 0;
        maFilled = true;
      }

      double maCount = maFilled ? MOVING_AVG_SIZE : maIndex;
      double maDistance = maSum / (maCount > 0 ? maCount : 1);

      // Spike detection: if distance deviates too much from moving average, treat as spike
      if (lastValidDistance >= 0 && fabs(distance - maDistance) > SPIKE_DISTANCE_THRESHOLD) {
        // ignore spike: use moving average instead
        distance = maDistance;
      }

      lastValidDistance = distance;

      if (filteredDistance < 0)
      {
        filteredDistance = distance;
      }
      else
      {
        filteredDistance = DISTANCE_FILTER_ALPHA * distance + (1.0 - DISTANCE_FILTER_ALPHA) * filteredDistance;
      }
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
  // Physics-based controller: distance error → ball acceleration
  // Distance error on beam is converted to desired ball acceleration via PID
  // Bereken fout (position error in cm)
  double controlDistance = -1.0;
  if (measurementSource == SOURCE_VISION) {
    controlDistance = (visionFilteredDistance >= 0) ? visionFilteredDistance : visionDistance;
  } else {
    controlDistance = (filteredDistance >= 0) ? filteredDistance : distance;
  }
  if (controlDistance < 0) {
    return lastPidOutput;
  }

  if (prevControlDistance < 0) {
    prevControlDistance = controlDistance;
  }

  static unsigned long lastControlUs = 0;
  unsigned long nowUs = micros();
  double dt = 0.015;
  if (lastControlUs != 0) {
    dt = (double)(nowUs - lastControlUs) / 1000000.0;
    dt = constrain(dt, 0.005, 0.05);
  }
  lastControlUs = nowUs;

  // Position error (cm)
  error = setpoint - controlDistance;
  if (fabs(error) < PID_ERROR_DEADBAND_CM) {
    error = 0.0;
  }

  // Convert cm -> m once and keep PID terms in compatible units.
  double errorM = error / 100.0;

  // Differentiate the measured distance (not error) to avoid setpoint kick
  double measurementDelta = controlDistance - prevControlDistance;
  prevControlDistance = controlDistance;

  measuredSpeed_cm_s = -measurementDelta / dt;
  filteredDerivative = PID_DERIVATIVE_ALPHA * filteredDerivative + (1.0 - PID_DERIVATIVE_ALPHA) * measuredSpeed_cm_s;

  integral += errorM * dt;
  integral = constrain(integral, -0.5, 0.5);
  prevError = error;

  // PID output: desired ball acceleration (m/s²)
  // Derivative is measured in cm/s; convert to m/s for consistent units.
  double derivativeM = filteredDerivative / 100.0;
  double rawOutput = Kp * errorM + Ki * integral + Kd * derivativeM;

  // Publish PID component contributions for telemetry / diagnostic plotting
  pid_p = Kp * errorM;      // proportional term (m/s^2)
  pid_i = Ki * integral;    // integral term (m/s^2)
  pid_d = Kd * derivativeM; // derivative term (m/s^2)
  pid_deriv_cm_s = filteredDerivative; // raw derivative in cm/s

  // When the ball is inside the deadband and essentially stopped, suppress control.
  // If it is still moving, keep the derivative term available for damping.
  if (fabs(error) < PID_ERROR_DEADBAND_CM && fabs(filteredDerivative) < SETTLE_DERIVATIVE_DEADBAND) {
    rawOutput = 0.0;
    integral *= 0.95;
  } else if (fabs(error) < SETTLE_ERROR_DEADBAND_CM && fabs(filteredDerivative) < SETTLE_DERIVATIVE_DEADBAND) {
    rawOutput = 0.0;
    integral *= 0.95;
  }

  // NOTE: The output-softening block near zero error was removed.
  // Multiplying rawOutput by 0.35 near the setpoint was suppressing the
  // derivative term right where damping matters most, causing oscillation.

  lastPidOutput = PID_OUTPUT_ALPHA * lastPidOutput + (1.0 - PID_OUTPUT_ALPHA) * rawOutput;
  
  // Limit acceleration command (±2 m/s² reasonable for 5.5g ball)
  return constrain(lastPidOutput, -1.0, 1.0);  // m/s²
}

void handleSerialCommand()
{
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    // Print safely: avoid passing Arduino String directly to printf (%s expects C string)
    Serial.print("serial      input: ");
    Serial.println(cmd);
    if (cmd.length() == 0) return;

    String cmdLower = cmd;
    cmdLower.toLowerCase();

    if (cmdLower == "auto") {
      manualServoOverride = false;
      Serial.println("Servo mode: AUTO");
      return;
    }

    if (cmdLower.startsWith("source:") || cmdLower.startsWith("input:")) {
      int sep = cmd.indexOf(':');
      // extract the token after the first colon and before any additional colon (e.g. "vision:-1")
      String rest = cmd.substring(sep + 1);
      int sep2 = rest.indexOf(':');
      String valueStr = (sep2 >= 0) ? rest.substring(0, sep2) : rest;
      valueStr.trim();
      valueStr.toLowerCase();

      if (valueStr == "vision" || valueStr == "cv" || valueStr == "camera") {
        measurementSource = SOURCE_VISION;
        Serial.println("Measurement source: VISION");
      } else if (valueStr == "sensor" || valueStr == "distance" || valueStr == "ir") {
        measurementSource = SOURCE_SENSOR;
        Serial.println("Measurement source: SENSOR");
      }
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
        manualServoAngle = constrain((int)round(valueStr.toFloat()), (int)round(servoMinDeg()), (int)round(servoMaxDeg()));
        manualServoOverride = true;
        Serial.printf("Servo mode: MANUAL | Angle: %d\n", manualServoAngle);
      }
        else if (param == "neutral") {
          SERVO_NEUTRAL_DEG = valueStr.toFloat();
          Serial.printf("Servo neutral updated to: %.1f\n", SERVO_NEUTRAL_DEG);
        }
        else if (param == "travel") {
          SERVO_TRAVEL_LIMIT_DEG = fmax(1.0, (double)valueStr.toFloat());
          Serial.printf("Servo travel updated to: %.1f\n", SERVO_TRAVEL_LIMIT_DEG);
        }
        else if (param == "dir") {
          CONTROL_DIRECTION = (valueStr.toFloat() >= 0.0) ? 1.0 : -1.0;
          Serial.printf("Control direction updated to: %.1f\n", CONTROL_DIRECTION);
        }
        else if (param == "servofilter") {
          SERVO_FILTER_ALPHA = constrain(valueStr.toFloat(), 0.0f, 1.0f);
          Serial.printf("Servo filter alpha updated to: %.3f\n", SERVO_FILTER_ALPHA);
        }
        else if (param == "servorate") {
          SERVO_RATE_LIMIT_DEG = fmax(0.05, (double)valueStr.toFloat());
          Serial.printf("Servo rate limit updated to: %.2f\n", SERVO_RATE_LIMIT_DEG);
        }
          else if (param == "breakaway") {
            BEAM_BREAKAWAY_DEG = fmax(0.0, (double)valueStr.toFloat());
            Serial.printf("Breakaway angle updated to: %.2f\n", BEAM_BREAKAWAY_DEG);
          }
        else if (param == "servodead") {
          SERVO_DEADBAND_DEG = fmax(0.0, (double)valueStr.toFloat());
          Serial.printf("Servo deadband updated to: %.2f\n", SERVO_DEADBAND_DEG);
        }
        else if (param == "piddead") {
          PID_ERROR_DEADBAND_CM = fmax(0.0, (double)valueStr.toFloat());
          Serial.printf("PID deadband updated to: %.2f\n", PID_ERROR_DEADBAND_CM);
        }
        else if (param == "settleerr") {
          SETTLE_ERROR_DEADBAND_CM = fmax(0.0, (double)valueStr.toFloat());
          Serial.printf("Settle error deadband updated to: %.2f\n", SETTLE_ERROR_DEADBAND_CM);
        }
        else if (param == "settlederiv") {
          SETTLE_DERIVATIVE_DEADBAND = fmax(0.0, (double)valueStr.toFloat());
          Serial.printf("Settle derivative deadband updated to: %.3f\n", SETTLE_DERIVATIVE_DEADBAND);
        }
        else if (param == "pidoutalpha") {
          PID_OUTPUT_ALPHA = constrain(valueStr.toFloat(), 0.0f, 1.0f);
          Serial.printf("PID output alpha updated to: %.3f\n", PID_OUTPUT_ALPHA);
        }
        else if (param == "derivalpha") {
          PID_DERIVATIVE_ALPHA = constrain(valueStr.toFloat(), 0.0f, 1.0f);
          Serial.printf("PID derivative alpha updated to: %.3f\n", PID_DERIVATIVE_ALPHA);
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
      else if (param == "vision" || param == "ball" || param == "pos") {
        updateExternalVisionDistance(valueStr.toFloat());
        // Automatically switch to vision input when external vision samples arrive
        measurementSource = SOURCE_VISION;
        Serial.println("Measurement source: VISION");
        Serial.printf("Vision distance updated to: %.2f\n", distance);
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

  // ===== CONTROL LAW: Distance Error → Acceleration → Beam Angle → Servo Angle =====
  // Step 1: PID computes desired ball acceleration from distance error (m/s²)
  desiredAccelerationMPS2 = PD_regelaar();  // m/s²

  // Step 2: Convert acceleration to beam angle using rolling dynamics
  // a = g * sin(alpha) / I_factor
  // => alpha = arcsin(a * I_factor / g)
  // Clamp to the beam range the servo can actually produce (about ±3.4°)
  double accelRatio = desiredAccelerationMPS2 * INERTIA_FACTOR / GRAVITY_M_S2;
  accelRatio = constrain(accelRatio, -1.0, 1.0);  // sin is bounded [-1, 1]
  desiredBeamAngleRad = asin(accelRatio);
  desiredBeamAngleDeg = desiredBeamAngleRad * (180.0 / M_PI);  // Convert to degrees
  desiredBeamAngleDeg = CONTROL_DIRECTION * desiredBeamAngleDeg;
  desiredBeamAngleDeg = constrain(desiredBeamAngleDeg, -maxBeamTiltDeg(), maxBeamTiltDeg());

  // Static friction / stiction compensation:
  // Only apply breakaway after the ball has been basically still for a while.
  // This avoids influencing normal motion and only helps it overcome static friction.
  bool ballNearlyStill = fabs(measuredSpeed_cm_s) < SETTLE_DERIVATIVE_DEADBAND;
  if (ballNearlyStill && fabs(error) >= PID_ERROR_DEADBAND_CM) {
      if (DEBUG) Serial.printf("ballNearlyStill %d, fabs(measuredSpeed_cm_s) %f\n", ballNearlyStill, fabs(measuredSpeed_cm_s));

    if (ballStillSinceMs == 0) {
      ballStillSinceMs = currentMillis;
    }
  } else {
    ballStillSinceMs = 0;
  }

  double activeBreakawayDeg = 0.0;
  if (ballStillSinceMs != 0) {
    unsigned long stillMs = currentMillis - ballStillSinceMs;
    // if (stillMs > BREAKAWAY_HOLD_MS * 4) {
      double ramp = (double)(stillMs - BREAKAWAY_HOLD_MS) / (double)BREAKAWAY_RAMP_MS;
      // ramp = constrain(ramp, 0.0, 10.0);
      activeBreakawayDeg = BEAM_BREAKAWAY_DEG * ramp;
    // }
  }

  if (ballNearlyStill && activeBreakawayDeg > 0.0 &&
      fabs(desiredBeamAngleDeg) > 0.0 &&
      fabs(desiredBeamAngleDeg) < activeBreakawayDeg) {
    desiredBeamAngleDeg = copysign(activeBreakawayDeg, desiredBeamAngleDeg);
    // Serial.printf("activeBreakawayDeg %lf, desiredBeamAngleDeg %lf\n", activeBreakawayDeg, desiredBeamAngleDeg);
  }

  output = desiredBeamAngleDeg;

  // Step 3: Convert beam angle to servo angle using lever kinematics
  // Servo neutral is calibrated at 84°; positive beam tilt maps above or below neutral
  // theta_servo = neutral + (alpha_beam / gain)
  double servoAngleFromBeam = desiredBeamAngleDeg / LEVER_TO_BEAM_GAIN;
  double autoServoAngle = SERVO_NEUTRAL_DEG + servoAngleFromBeam;
  autoServoAngle = constrain(autoServoAngle, servoMinDeg(), servoMaxDeg());

  // Smooth the servo target a little so tiny control changes do not turn into jitter.
  double targetServoAngle = manualServoOverride
      ? constrain((double)manualServoAngle, servoMinDeg(), servoMaxDeg())
      : autoServoAngle;

  if (!filteredServoAngleInitialized) {
    filteredServoAngle = targetServoAngle;
    filteredServoAngleInitialized = true;
  } else {
    double filteredTarget = SERVO_FILTER_ALPHA * targetServoAngle + (1.0 - SERVO_FILTER_ALPHA) * filteredServoAngle;
    double maxStep = max(0.05, SERVO_RATE_LIMIT_DEG);
    double delta = constrain(filteredTarget - filteredServoAngle, -maxStep, maxStep);
    filteredServoAngle = constrain(filteredServoAngle + delta, servoMinDeg(), servoMaxDeg());
  }

  loopCounter++;

  if (millis() - loopTimer >= 1000)
  {
    savedCount = loopCounter;
    savedAdcCount = adcCount;
    savedControlCount = controlCount;
    loopCounter = 0;
    adcCount = 0;
    controlCount = 0;
    loopTimer = millis();
  }

  int servoAngle = (int)round(filteredServoAngle);

  // Do not send tiny corrections that only excite servo backlash/noise.
  double writeThresholdDeg = max((double)SERVO_WRITE_MIN_STEP_DEG, SERVO_DEADBAND_DEG);
  if ((fabs(filteredServoAngle - (double)lastWrittenServoAngle) >= writeThresholdDeg) && !(fabs(error) < SETTLE_ERROR_DEADBAND_CM && fabs(measuredSpeed_cm_s) < SETTLE_DERIVATIVE_DEADBAND)) {
    controlCount++;
    lastWrittenServoAngle = servoAngle;
    writeServoAngle(servoAngle);
    if (DEBUG) Serial.printf("writeServoAngle\t%f\t%f\n",fabs(error), fabs(measuredSpeed_cm_s));

  }
  else
  {
    if (DEBUG) Serial.printf("NOT writeServoAngle\t%f\t%f\n",fabs(error), fabs(measuredSpeed_cm_s));

  }
  if (ballNearlyStill) {
    if (DEBUG) Serial.printf("servoAngle %d\n", servoAngle);
  }


  if (currentMillis - lastTelemetryMs >= telemetryIntervalMs) {
    lastTelemetryMs = currentMillis;
    // Plot-friendly line: full physics-based control chain
    // Out = beam command in degrees, Servo = actual servo angle in degrees
    Serial.printf("Plot,Mode:%d,Src:%d,Set:%.2f,Raw:%d,Volt:%.3f,Dist:%.2f,DistF:%.2f,Speed:%.3f,Err:%.2f,Accel:%.2f,POut:%.3f,DOut:%.3f,IOut:%.3f,Deriv:%.3f,Out:%.2f,Servo:%.2f,Kp:%.2f,Ki:%.3f,Kd:%.2f,Neutral:%.1f,Travel:%.1f,Dir:%.1f,PidDead:%.2f,SettleErr:%.2f,SettleDeriv:%.3f,ServoFilt:%.3f,ServoRate:%.2f,Breakaway:%.2f,ServoDead:%.2f,LoopCount:%lu,ADC:%lu,Control:%lu\n",
                    manualServoOverride ? 1 : 0, measurementSource == SOURCE_VISION ? 1 : 0, setpoint, sensorRaw, voltage, distance, filteredDistance, measuredSpeed_cm_s, error, desiredAccelerationMPS2,
                    pid_p, pid_d, pid_i, pid_deriv_cm_s, output, servoAngle, Kp, Ki, Kd, SERVO_NEUTRAL_DEG, SERVO_TRAVEL_LIMIT_DEG, CONTROL_DIRECTION, PID_ERROR_DEADBAND_CM, SETTLE_ERROR_DEADBAND_CM, SETTLE_DERIVATIVE_DEADBAND, SERVO_FILTER_ALPHA, SERVO_RATE_LIMIT_DEG, BEAM_BREAKAWAY_DEG, SERVO_DEADBAND_DEG, savedCount, savedAdcCount, savedControlCount);
  }
}