import os
import json
import time
import paho.mqtt.client as mqtt

# MQTT Broker configuration
mqtt_broker = "mqtt.example.com"  # Replace with your MQTT broker address
mqtt_port = 1883

# Directory for storing data files
data_directory = "/home/pi/mqtt_data"  # Replace with your desired data directory path

# Callback function for MQTT connection
def on_connect(client, userdata, flags, rc):
    print("Connected to MQTT broker with result code " + str(rc))
    # Subscribe to MQTT topics for device data
    client.subscribe("devices/+/+")

# Callback function for MQTT message
def on_message(client, userdata, msg):
    print("Received MQTT message on topic " + msg.topic + ": " + str(msg.payload))
    
    # Extract device ID and metric type from MQTT topic
    topic_parts = msg.topic.split("/")
    device_id = topic_parts[1]
    metric_type = topic_parts[2]

    # Create directory for device if not exists
    device_directory = os.path.join(data_directory, device_id)
    os.makedirs(device_directory, exist_ok=True)

    # Write message payload to data file
    data_file_path = os.path.join(device_directory, f"{metric_type}.txt")
    with open(data_file_path, "a") as f:
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        data = {"timestamp": timestamp, "value": msg.payload.decode("utf-8")}
        json.dump(data, f)
        f.write("\n")

# Initialize MQTT client
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

# Connect to MQTT broker
client.connect(mqtt_broker, mqtt_port, 60)

# MQTT loop (infinite loop to handle network and callbacks)
client.loop_forever()

