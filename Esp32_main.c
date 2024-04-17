#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

// WiFi credentials
const char* ssid = "YourWiFiSSID";     // Replace with your WiFi network SSID
const char* password = "YourWiFiPassword"; // Replace with your WiFi network password

// MQTT Broker (Server) configuration
const char* mqttBroker = "mqtt.example.com"; // MQTT Broker address
const int mqttPort = 1883;                    // MQTT Broker port

// MQTT topic for publishing data
const char* mqttTopic = "sensor/data";       // MQTT topic to publish data

// MQTT client instance
WiFiClient espClient;
PubSubClient mqttClient(espClient);

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

  // Set MQTT server and port
  mqttClient.setServer(mqttBroker, mqttPort);

  // Configure ADC for 12-bit resolution (0-4095) for higher precision
  analogReadResolution(12);
}

void loop() {
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }

  mqttClient.loop();

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

    // Create payload with current and voltage data
    String payload = String(currentAmps) + "," + String(voltageVolts);

    // Publish payload to MQTT topic
    mqttClient.publish(mqttTopic, payload.c_str());

    Serial.println("Published to MQTT:");
    Serial.println(payload);
  }
}

void reconnectMQTT() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    
    // Attempt to connect
    if (mqttClient.connect("ESP32Client")) {
      Serial.println("Connected to MQTT Broker");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" Retrying in 5 seconds...");
      
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
