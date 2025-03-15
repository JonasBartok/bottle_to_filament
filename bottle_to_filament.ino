#include <thermistor.h>

#define THERMISTOR_PIN A0
#define HEATER_PIN 9
#define TARGET_TEMP 250  // in °C
#define TEMP_TOLERANCE 5
#define TERMISTOR_OHM 100000

float currentTemp = 0;
float error, previousError;
float Kp = 2.0, Ki = 0.1, Kd = 1.0;
float integral = 0, derivative = 0;

thermistor therm1(THERMISTOR_PIN, 0);

void setup() {
  Serial.begin(115200);
  pinMode(HEATER_PIN, OUTPUT);
}

void loop() {
  float currentTemp = therm1.analog2temp();
  controlHeater();
  delay(500);
}

void controlHeater() {
  error = TARGET_TEMP - currentTemp;  
  integral += error;                  
  derivative = error - previousError;  

  // PID Calculation
  float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  output = constrain(output, 0, 255);  

  analogWrite(HEATER_PIN, output);  
}
