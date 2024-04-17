#include <Arduino.h>
#include <WiFi.h>

// WiFi credentials
const char* ssid = "YourWiFiSSID";     // Replace with your WiFi network SSID
const char* password = "YourWiFiPassword"; // Replace with your WiFi network password

// Define analog input pins for current and voltage measurements
const int currentSensorPin = 34;   // Analog input pin for current sensor
const int voltageSensorPin = 35;   // Analog input pin for voltage sensor

// Define measurement interval (in milliseconds)
const unsigned long measurementInterval = 5000; // Interval of 5 seconds

void setup() {
  Serial.begin(115200); // Initialize serial communication
  delay(100);

  // Connect to WiFi
  Serial.println();
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected successfully");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Configure ADC for 12-bit resolution (0-4095) for higher precision
  analogReadResolution(12);
}

void loop() {
  static unsigned long lastMeasurementTime = 0;

  // Check if it's time to take a new measurement
  if (millis() - lastMeasurementTime >= measurementInterval) {
    lastMeasurementTime = millis(); // Update last measurement time

    // Read analog values from current and voltage sensors
    int currentRawValue = analogRead(currentSensorPin);
    int voltageRawValue = analogRead(voltageSensorPin);

    // Convert raw ADC values to physical units (current in Amps, voltage in Volts)
    float currentAmps = map(currentRawValue, 0, 4095, 0, 30) / 1000.0; // Example: Map 0-4095 to 0-30 Amps
    float voltageVolts = map(voltageRawValue, 0, 4095, 0, 3300) / 1000.0; // Example: Map 0-4095 to 0-3.3 Volts

    // Print current and voltage readings
    Serial.print("Current: ");
    Serial.print(currentAmps, 3); // Print current value with 3 decimal places
    Serial.print(" A, Voltage: ");
    Serial.print(voltageVolts, 2); // Print voltage value with 2 decimal places
    Serial.println(" V");
  }

  // Add any other code that needs to run continuously here
}
