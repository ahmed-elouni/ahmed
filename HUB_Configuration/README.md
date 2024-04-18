***Instructions****:
Replace mqtt_broker with the IP address or hostname of your MQTT broker.

Define data_directory to specify the directory path where you want to store the data files.

Run the Python script on your Raspberry Pi:


python mqtt_data_logger.py

***How it Works****:
The script sets up an MQTT client using the paho-mqtt library to connect to your MQTT broker.
It defines callback functions (on_connect and on_message) to handle MQTT connection events (e.g., subscription) and incoming MQTT messages.
Upon receiving a message on any MQTT topic (devices/+/+), the script extracts the device ID and metric type from the topic.
It creates a directory for each device under the specified data_directory if it does not already exist.
The script writes the received message payload (data value) along with a timestamp to a JSON-formatted data file (<metric_type>.txt) within the device's directory.

***Testing****:

Publish MQTT messages to topics like devices/device123/current or devices/device456/voltage from your ESP32 or other MQTT clients.
Observe the directories and data files being created under data_directory on your Raspberry Pi.

Verify the content of the data files to ensure that incoming MQTT messages are being stored correctly.

This Python script acts as an MQTT data logger on the Raspberry Pi, continuously listening for MQTT messages, parsing the data, and storing it in structured files based on the source device and metric type.
