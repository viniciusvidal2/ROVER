import paho.mqtt.client as mqtt
import json

class MqttHandler:
    def __init__(self, broker, port, topic):
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.topic = topic
        self.client.on_message = self.on_message
        self.client.reconnect_delay_set(min_delay=1, max_delay=120)

        try:
            self.client.connect(broker, port, 0)
            
            self.client.subscribe(f"{topic}/commands")
            
            self.client.loop_start()
        except Exception as e:
            print("Error init MQTT {e}")
            
        self.flag_gps = False
    
    def __del__(self):
        self.client.loop_stop()
        self.client.disconnect()
    
    def on_message(self, client, userdata, message):
        if message.topic == f"{self.topic}/commands":
            msg = json.loads(message.payload.decode())
            if msg.get("gps") == 1:
                self.flag_gps = True
                
    
    def publish_telemetry(self, temperature, battery, speed, gps_coordinates, status):
        try:
            sum_temperatures = 0
            for data in temperature.values():
                sum_temperatures += data["temperature"]
            message = {
                "battery": battery,
                "temperature": f"{sum_temperatures/len(temperature):.2f}",
                "speed": speed,
                "location": {
                    "lat": gps_coordinates["latitude"],
                    "lng": gps_coordinates["longitude"],
                },
                "status": status,
            }
            self.client.publish(f'{self.topic}/telemetry', json.dumps(message))
        except Exception as e:
            print("Publish Telemetry failed {e}")
        
    def publish_gps(self, gps, compass):
        try:
            message = {
                "lat": gps["latitude"],
                "lon": gps["longitude"],
                "compass": compass
            }
            self.client.publish(f'{self.topic}/escolha', json.dumps(message))
        except Exception as e:
            print("Publish GPS failed {e}")