#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

// WiFi credentials
const char* ssid = "YourWiFiSSID";     // Replace with your WiFi network SSID
const char* password = "YourWiFiPassword"; // Replace with your WiFi network password

// MQTT Broker (Server) configuration
const char* mqttBroker = "mqtt.example.com"; // MQTT Broker address
const int mqttPort = 1883;                    // MQTT Broker port

// MQTT client instance
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Define analog input pins for current and voltage measurements
const int currentSensorPin = 34;   // Analog input pin for current sensor
const int voltageSensorPin = 35;   // Analog input pin for voltage sensor

// Define relay pin
const int relayPin = 26;  // GPIO pin connected to the relay control

// Define measurement interval (in milliseconds)
const unsigned long measurementInterval = 5000; // Interval of 5 seconds

// Thresholds for current and voltage (in Amps and Volts)
const float currentThreshold = 0.5;  // Threshold current (in Amps) to trigger the relay
const float voltageThreshold = 2.0;  // Threshold voltage (in Volts) to trigger the relay

// Function to get unique device ID based on ESP32 chip ID
String getDeviceId() {
  uint64_t chipId = ESP.getEfuseMac(); // Get ESP32 chip ID
  return String(chipId, HEX); // Convert chip ID to hexadecimal string
}

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

  // Set relay pin as output
  pinMode(relayPin, OUTPUT);
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

    // Get unique device ID (ESP32 chip ID)
    String deviceId = getDeviceId();

    // Publish current and voltage data to MQTT topics
    publishToMQTT(deviceId, "current", currentAmps);
    publishToMQTT(deviceId, "voltage", voltageVolts);

    // Check if current or voltage exceeds thresholds
    if (currentAmps > currentThreshold || voltageVolts > voltageThreshold) {
      // Turn on the relay (active LOW)
      digitalWrite(relayPin, LOW);
      Serial.println("Relay ON");

      // Publish relay status (ON) to MQTT topic
      publishToMQTT(deviceId, "relay_status", 1); // Publish 1 (ON) for relay status
    } else {
      // Turn off the relay (active LOW)
      digitalWrite(relayPin, HIGH);
      Serial.println("Relay OFF");

      // Publish relay status (OFF) to MQTT topic
      publishToMQTT(deviceId, "relay_status", 0); // Publish 0 (OFF) for relay status
    }
  }
}

void reconnectMQTT() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    
    // Attempt to connect
    if (mqttClient.connect(getDeviceId().c_str())) {
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

void publishToMQTT(const char* deviceId, const char* metricType, float value) {
  // Construct MQTT topic based on device ID and metric type
  String topic = "devices/" + String(deviceId) + "/" + String(metricType);

  // Convert float value to string for MQTT payload
  String payload = String(value, 2); // 2 decimal places

  // Publish payload to MQTT topic
  mqttClient.publish(topic.c_str(), payload.c_str());

  Serial.print("Published to MQTT - Topic: ");
  Serial.print(topic);
  Serial.print(", Payload: ");
  Serial.println(payload);
}
