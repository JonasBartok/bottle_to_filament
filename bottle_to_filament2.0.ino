#include <math.h>

// Pin setup
const int analogPin = A0;      // Thermistor input
const int mosfetPin = 3;       // PWM output to MOSFET

// Voltage & resistors
const float Vcc = 5.0;
const float R_fixed = 100000.0; // 100k

// Thermistor
const float R0 = 100000.0;
const float T0 = 298.15; // Kelvin
const float Beta = 3950.0;

// PID target
float setpoint = 100.0; // Target temp in °C

// PID constants (tune these!)
float Kp = 50.0, Ki = 0.5, Kd = 0.001;

// PID state
float integral = 0.0;
float lastError = 0.0;
unsigned long lastTime = 0;

void setup() {
  Serial.begin(9600);
  pinMode(mosfetPin, OUTPUT);
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0; // Time step in seconds
  if (dt <= 0.0) dt = 0.01;
  lastTime = now;

  // --- Read temperature ---
  int analogValue = analogRead(analogPin);
  float Vout = analogValue * (Vcc / 1023.0);
  if ((Vcc - Vout) < 0.01) return; // avoid div by zero
  float R_thermistor = (Vout * R_fixed) / (Vcc - Vout);
  float tempK = 1.0 / ((1.0 / T0) + (1.0 / Beta) * log(R_thermistor / R0));
  float tempC = tempK - 273.15;

  // --- PID ---
  float error = setpoint - tempC;
  integral += error * dt;
  float derivative = (error - lastError) / dt;
  float output = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;

  // Clamp output to 0–255
  output = constrain(output, 0, 255);
  analogWrite(mosfetPin, (int)output);

  // --- Debug ---
  Serial.print("Temp: ");
  Serial.print(tempC, 2);
  Serial.print(" °C | Output: ");
  Serial.print((int)output);
  Serial.print("\n");
  delay(100); // Optional: slow loop slightly
}
