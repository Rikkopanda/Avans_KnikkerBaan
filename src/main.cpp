#include <Arduino.h>
#include <Servo.h>
#define dist_sensor A1

Servo myservo;  // Create servo object
int potpin = A0;  // Potentiometer connected to A0

int val;          // Variable to store potentiometer value
unsigned long previousMillis = 0;
const long interval_write_servo = 15;  // Update every 15ms instead of delay(15)
double prevError = 0;
double Kp = 10;
double Kd = 2;
double distance;
int setpoint;


void setup()
{  
  Serial.begin(115200);
  myservo.attach(9);  // Attach servo to pin 9
}

void leesSensorEnPot()
{
  // beter avarage nemen van aantal meet punten??
  int sensorValue = analogRead(dist_sensor);
  float voltage = sensorValue * (5.0 / 1023.0);

  // int distance = 13 * pow(voltage, -1); // Derived from datasheet graph
  distance = 2076.0 / (sensorValue - 11.0);

  if (distance <= 30) {
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
  } else {
    Serial.println("Out of range");
  }
  setpoint = analogRead(potpin);
  setpoint = map(setpoint, 0, 1023, 4, 30);
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

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    myservo.write(servoAngle);
  }

  // Debug
  Serial.print("Setpoint: "); Serial.print(setpoint);
  Serial.print(" | Distance: "); Serial.print(distance);
  Serial.print(" | Angle: "); Serial.println(servoAngle);
}
