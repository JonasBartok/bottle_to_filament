#define THERMISTOR_PIN A0
#define HEATER_PIN 9
#define TARGET_TEMP 100  // in °C 250
#define TEMP_TOLERANCE 5

// Constants
const int analogPin = A0;       // Analog input pin
const float Vcc = 5.0;          // Supply voltage
const float R_fixed = 100000.0; // 100k ohm fixed resistor

// Thermistor parameters
const float R0 = 100000.0;      // Resistance at 25°C (100k ohm)
const float T0 = 298.15;        // 25°C in Kelvin
const float Beta = 3950.0;      // Beta parameter (check your thermistor's datasheet)

float currentTemp = 0;
float error, previousError;
float Kp = 25.0, Ki = 0.5, Kd = 0.001;
float integral = 0, derivative = 0;

void setup() {
  Serial.begin(9600);
  pinMode(HEATER_PIN, OUTPUT);
}

void loop() {
  float currentTemp = calculateTemp();
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

void heater() {
  if (currentTemp < TARGET_TEMP) {
    digitalWrite(HEATER_PIN, HIGH);
  } else {
    digitalWrite(HEATER_PIN, LOW);
  }
}

float calculateTemp() {
  // Read analog value
  int analogValue = analogRead(THERMISTOR_PIN);

  // Convert to voltage
  float Vout = analogValue * (Vcc / 1023.0);

  // Calculate thermistor resistance (assuming thermistor is connected to GND)
  float R_thermistor = (Vout * R_fixed) / (Vcc - Vout);

  // Calculate temperature using the Beta formula
  float temperatureK = 1.0 / ( (1.0 / T0) + (1.0 / Beta) * log(R_thermistor / R0) );
  float temperatureC = temperatureK - 273.15;

  // Print results
  Serial.print("Temperature: ");
  Serial.print(temperatureC);
  Serial.println(" °C");

  return temperatureC; 
}
