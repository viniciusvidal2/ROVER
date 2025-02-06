import paho.mqtt.client as mqtt
import json
from lewansoul_servo_bus import ServoBus

PAN_SERVO_ID = 1
TILT_SERVO_ID = 2
PAN_VALUES = 15
TILT_VALUES = 90
PAN_PIXELS = 1280  # Largura da imagem
TILT_PIXELS = 720  # Altura da imagem
PAN_FOV = 120
TILT_FOV = 60

class MqttHandler:
    def __init__(self, broker, port, topic):
        self.init_mqtt(broker, port, topic)
        
        self.flag_gps = False
        
        self.init_servo()
    
    def init_mqtt(self, broker, port, topic):
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.topic = topic
        self.client.on_message = self.on_message
        self.client.reconnect_delay_set(min_delay=1, max_delay=120)

        try:
            self.client.connect(broker, port, 0)
            
            self.client.subscribe(f"{topic}/commands")
            self.client.subscribe(f"{topic}/escolha")
            
            self.client.loop_start()
        except Exception as e:
            print("Error init MQTT {e}")
    
    def init_servo(self):
        self.servo = ServoBus("/dev/ttyUSB0")

        # Ângulos iniciais "standby"
        self.PAN_STANDBY_ANGLE = self.servo.pos_read(PAN_SERVO_ID)
        self.servo.move_time_write(1, 125, 2)
        self.TILT_STANDBY_ANGLE = self.servo.pos_read(TILT_SERVO_ID)
        self.servo.move_time_write(2, 240, 2)
        
        # Limites
        self.PAN_MIN_ANGLE = self.PAN_STANDBY_ANGLE - PAN_VALUES
        self.PAN_MAX_ANGLE = self.PAN_STANDBY_ANGLE + PAN_VALUES
        self.TILT_MIN_ANGLE = self.TILT_STANDBY_ANGLE - TILT_VALUES
        self.TILT_MAX_ANGLE = self.TILT_STANDBY_ANGLE + TILT_VALUES
    
    def __del__(self):
        self.client.loop_stop()
        self.client.disconnect()
    
    def on_message(self, client, userdata, message):
        msg = json.loads(message.payload.decode())
        if message.topic == f"{self.topic}/commands":
            self.subscription_commands(msg)
        elif message.topic == f"{self.topic}/escolha":
            self.subscription_escolha(msg)
    
    def subscription_commands(self, msg):
        if msg.get("gps") == 1:
            self.flag_gps = True
    
    def subscription_escolha(self, msg):
        deltaX = msg.get("deltaX", 0)
        deltaY = msg.get("deltaY", 0)
        
        pan_angle = self.pixels_to_angle(deltaX, PAN_PIXELS, PAN_FOV)
        tilt_angle = self.pixels_to_angle(deltaY, TILT_PIXELS, TILT_FOV)
        
        # Ajusta com base no ângulo "standby"
        pan_angle = -pan_angle + self.PAN_STANDBY_ANGLE
        tilt_angle = tilt_angle + self.TILT_STANDBY_ANGLE
        
        # Respeita limites
        pan_angle = max(self.PAN_MIN_ANGLE, min(self.PAN_MAX_ANGLE, pan_angle))
        tilt_angle = max(self.TILT_MIN_ANGLE, min(self.TILT_MAX_ANGLE, tilt_angle))
        
        self.set_servo_angle(PAN_SERVO_ID, pan_angle, 2)
        self.set_servo_angle(TILT_SERVO_ID, tilt_angle, 2)
    
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
            
    def pixels_to_angle(self, pixels, max_pixels, fov):
        """Converte um número de pixels em ângulos respeitando os limites."""
        angle = (pixels / max_pixels) * fov
        return -angle


    def set_servo_angle(self, servo_id, angle, duration=2):
        """Define o ângulo do servo especificado com um tempo de movimento."""
        if 0 <= angle <= 240:  # Limite de angulo permitido pela lib
            self.servo.move_time_write(servo_id, angle, duration)
        else:
            print(f"Erro: Ângulo {angle} fora do limite permitido (0-240 graus).")