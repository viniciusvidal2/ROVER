import paho.mqtt.client as mqtt
import json
from lewansoul_servo_bus import ServoBus

############################################################################
# region Constants
############################################################################

# Servo
PAN_SERVO_ID = 1
TILT_SERVO_ID = 2
PAN_STANDBY_ANGLE = 125
TILT_STANDBY_ANGLE = 240
PAN_VALUES = 15
TILT_VALUES = 80
PAN_MIN_ANGLE = PAN_STANDBY_ANGLE - PAN_VALUES
PAN_MAX_ANGLE = PAN_STANDBY_ANGLE + PAN_VALUES
TILT_MIN_ANGLE = TILT_STANDBY_ANGLE - TILT_VALUES
TILT_MAX_ANGLE = TILT_STANDBY_ANGLE + TILT_VALUES
PAN_PIXELS = 1280  # Image width
TILT_PIXELS = 720  # Image height
PAN_FOV = 120
TILT_FOV = 60

# endregion

class MqttHandler:
    ############################################################################
    # region Init and Close
    ############################################################################

    def __init__(self, broker, port, topic) -> None:
        """Initialize the MQTT handler with broker connection, servo setup and other configurations.

        Args:
            broker (str): MQTT broker address
            port (int): MQTT broker port
            topic (str): Base MQTT topic for communications
        """
        # flags initialization
        self.flag_gps = False
        self.flag_lantern = -1
        
        self.init_mqtt(broker, port, topic)
        
        self.init_servo()

    def init_mqtt(self, broker, port, topic) -> None:
        """
        Initialize MQTT client connection and subscriptions.

        Args:
            broker (str): MQTT broker address
            port (int): MQTT broker port
            topic (str): Base MQTT topic for communications
        """
        try:
            self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
            self.topic = topic
            self.client.on_message = self.on_message
            self.client.reconnect_delay_set(min_delay=1, max_delay=120)

            self.client.connect(broker, port, 0)

            self.client.subscribe(f"{topic}/commands")
            self.client.subscribe(f"{topic}/escolha")

            self.client.loop_start()
        except Exception as e:
            print("Error init MQTT {e}")

    def init_servo(self) -> None:
        """
        Initialize servo motors by connecting to USB port and setting standby positions.
        Sets initial pan and tilt angles to their respective standby positions.
        """
        self.pan = 0
        self.tilt = 0
        
        try:
            self.servo = ServoBus("/dev/ttyUSB0")

            self.servo.move_time_write(1, PAN_STANDBY_ANGLE, 2)
            self.servo.move_time_write(2, TILT_STANDBY_ANGLE, 2)

        except Exception as e:
            print("Error init Servo {e}")

    def close(self) -> None:
        """
        Cleanup method to properly stop MQTT client loop and disconnect from broker.
        """
        self.client.loop_stop()
        self.client.disconnect()

        if hasattr(self, "servo"):
            self.servo.move_time_write(PAN_SERVO_ID, PAN_STANDBY_ANGLE, 2)
            self.servo.move_time_write(TILT_SERVO_ID, TILT_STANDBY_ANGLE, 2)

    # endregion

    ############################################################################
    # region Subscribe
    ############################################################################

    def on_message(self, client, userdata, message) -> None:
        """
        MQTT message callback handler. Routes messages to appropriate subscription handlers.

        Args:
            client (mqtt.Client): MQTT client instance
            userdata (any): User defined data passed to callback
            message (mqtt.MQTTMessage): Received MQTT message
        """
        msg = json.loads(message.payload.decode())
        if message.topic == f"{self.topic}/commands":
            self.subscription_commands(msg)
        elif message.topic == f"{self.topic}/escolha":
            self.subscription_escolha(msg)
        elif message.topic == f"{self.topic}/lantern":
            self.subscription_lantern(msg)
    
    def subscription_commands(self, msg) -> None:
        """
        Handle command messages received on the commands topic.
        
        Args:
            msg (dict): JSON message containing command data
        """
        if msg.get("gps") == 1:
            self.flag_gps = True
    
    def subscription_escolha(self, msg) -> None:
        """
        Handle position adjustment messages for servo control.
        Calculates and sets new pan and tilt angles based on pixel deltas.

        Args:
            msg (dict): JSON message containing deltaX and deltaY values
        """
        deltaX = msg.get("deltaX", 0)
        deltaY = msg.get("deltaY", 0)
        
        if deltaX == 10000:
            self.pan = 0
            self.tilt = 0
            deltaX = 0
            deltaY = 0
            
        pan_angle = self.pixels_to_angle(deltaX, PAN_PIXELS, PAN_FOV)
        tilt_angle = self.pixels_to_angle(deltaY, TILT_PIXELS, TILT_FOV)
        self.pan += pan_angle
        self.tilt += tilt_angle
        
        # Adjust based on standby angle
        pan_angle = self.pan + PAN_STANDBY_ANGLE
        tilt_angle = -self.tilt + TILT_STANDBY_ANGLE
        
        # Respect movement limits
        pan_angle = max(PAN_MIN_ANGLE, min(PAN_MAX_ANGLE, pan_angle))
        tilt_angle = max(TILT_MIN_ANGLE, min(TILT_MAX_ANGLE, tilt_angle))
        
        self.set_servo_angle(PAN_SERVO_ID, pan_angle, 2)
        self.set_servo_angle(TILT_SERVO_ID, tilt_angle, 2)
    
    def subscription_lantern(self, msg) -> None:
        """
        Handle lantern control messages.
        
        Args:
            msg (dict): JSON message containing lantern state ('l' key)
        """
        self.flag_lantern = msg.get("l", -1)

    # endregion

    ############################################################################
    # region Publish
    ############################################################################

    def publish_telemetry(
        self, temperatures, battery, speed, gps_coordinates, gps_compass, status, lantern
    ) -> None:
        """
        Publish telemetry data to MQTT broker.

        Args:
            temperatures (dict): Temperature readings from sensors
            battery (float): Battery level
            speed (float): Current speed
            gps_coordinates (dict): GPS coordinates containing latitude and longitude
            gps_compass (float): Compass heading
            status (str): Current status
            lantern (int): Lantern state
        """
        try:
            message = {
                **temperatures,
                "battery": battery,
                "speed": speed,
                "location": {
                    "lat": gps_coordinates["latitude"],
                    "lng": gps_coordinates["longitude"],
                    "compass": gps_compass,
                },
                "status": status,
                "lantern": lantern,
            }
            self.client.publish(f"{self.topic}/telemetry", json.dumps(message))
        except Exception as e:
            print("Publish Telemetry failed {e}")

    def publish_gps(self, gps, compass) -> None:
        """
        Publish GPS location and compass data to MQTT broker.

        Args:
            gps (dict): GPS coordinates containing latitude and longitude
            compass (float): Compass heading
        """
        try:
            message = {
                "lat": gps["latitude"],
                "lon": gps["longitude"],
                "compass": compass,
            }
            self.client.publish(f"{self.topic}/escolha", json.dumps(message))
        except Exception as e:
            print("Publish GPS failed {e}")

    # endregion

    ############################################################################
    # region Auxiliary
    ############################################################################

    def pixels_to_angle(self, pixels, max_pixels, fov) -> None:
        """
        Convert pixel coordinates to angle values respecting limits.

        Args:
            pixels (int): Number of pixels to convert
            max_pixels (int): Maximum number of pixels in the dimension
            fov (float): Field of view in degrees

        Returns:
            float: Calculated angle in degrees
        """
        angle = (pixels / max_pixels) * fov
        return -angle

    def set_servo_angle(self, servo_id, angle, duration=2) -> None:
        """
        Set the angle for a specified servo motor with movement duration.

        Args:
            servo_id (int): ID of the servo motor
            angle (float): Target angle in degrees
            duration (int, optional): Movement duration in seconds. Defaults to 2.
        """
        if 0 <= angle <= 240:  # Limite de angulo permitido pela lib
            self.servo.move_time_write(servo_id, angle, duration)
        else:
            print(f"Erro: Ângulo {angle} fora do limite permitido (0-240 graus).")