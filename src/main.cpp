#include <Arduino.h>

// ===== SERVO PWM SETUP (ESP32 LEDC) =====
const int SERVO_PIN = 27;          // GPIO 27 for servo signal
const int SERVO_CHANNEL = 0;       // PWM channel 0
const int SERVO_FREQ_HZ = 50;      // 50 Hz for servo
const int SERVO_RES_BITS = 16;     // 16-bit resolution

// ===== SENSOR SELECTION =====
// Uncomment one of the following to select sensor type:
#define USE_HC_SR04        // HC-SR04 ultrasonic sensor
// #define USE_ANALOG_SENSOR  // Original analog distance sensor on GPIO A1

#ifdef USE_HC_SR04
  const int TRIG_PIN = 25;   // GPIO 25 for HC-SR04 trigger
  const int ECHO_PIN = 33;   // GPIO 33 for HC-SR04 echo
#else
  #define dist_sensor A1    // Analog sensor on A1
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

int potpin = A0;  // Potentiometer connected to A0

int val;          // Variable to store potentiometer value
unsigned long previousMillis = 0;
const long interval_write_servo = 15;  // Update every 15ms instead of delay(15)
#ifdef USE_HC_SR04
  const long RANGE_READ_INTERVAL_MS = 65; // HC-SR04 needs ~60ms between measurements
  unsigned long lastRangeReadMs = 0;
#endif
double prevError = 0;
double Kp = 10;
double Kd = 2;
double distance;
int setpoint;
double output;  // PD controller output


void setup()
{  
  Serial.begin(115200);
  
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
    int sensorValue = analogRead(dist_sensor);
    float voltage = sensorValue * (5.0 / 1023.0);
    distance = 2076.0 / (sensorValue - 11.0);

    if (distance <= 30) {
      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.println(" cm");
    } else {
      Serial.println("Out of range");
    }
  #endif
  
  // setpoint = analogRead(potpin);
  // setpoint = map(setpoint, 0, 1023, 4, 30);
  setpoint = 5;
}

double PD_regelaar()
{
  // Bereken fout
  double error = setpoint - distance;
  double derivative = error - prevError;
  prevError = error;

  // PD uitgang
  double output = Kp * error + Kd * derivative;
  return output;
}

void loop()
{
  unsigned long currentMillis = millis();

  if (Serial.available())
  {
      char buf[30];
      int readCnt = Serial.readBytes(buf, 30);
      buf[readCnt] = '\0';
      Serial.printf("String = %s\n", buf);
  }

  leesSensorEnPot();

  output = constrain(PD_regelaar(), -90, 90); // Beperk beweging

  // Zet om naar servo positie (0-180 graden)
  int servoAngle = map(output, -90, 90, 0, 180);

  if (currentMillis - previousMillis >= interval_write_servo) {
    previousMillis = currentMillis;
    writeServoAngle(servoAngle);
  }

  // Debug
  Serial.print("output: "); Serial.print(output);
  Serial.print(" | Setpoint: "); Serial.print(setpoint);
  Serial.print(" | Distance: "); Serial.print(distance);
  Serial.print(" | Angle: "); Serial.println(servoAngle);
}
