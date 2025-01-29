import board
from digitalio import DigitalInOut, Direction
import adafruit_dht
import adafruit_bmp280
import yaml
import rospy
from std_msgs.msg import Float32
import rospkg
import os

rospack = rospkg.RosPack()
config_path = os.path.join(
    rospack.get_path("temperature_sensors"), "config/sensors.yaml"
)

dht22_dict = {}

spi = board.SPI()
bmp280_dict = {}

dht22_temperature_publishers = {}
dht22_humidity_publishers = {}
bmp280_temperature_publishers = {}


def initialize_sensors():
    with open(config_path, "r") as file:
        sensors_list = yaml.safe_load(file)

        for sensor in sensors_list["bmp280"]:
            try:
                bmp280_cs = DigitalInOut(getattr(board, sensor["cs_pin"]))
                bmp280_cs.direction = Direction.OUTPUT
                bmp280_cs.value = 1
                bmp280 = adafruit_bmp280.Adafruit_BMP280_SPI(spi, bmp280_cs)
                bmp280_dict[sensor["name"]] = bmp280

                bmp280_temperature_publishers[sensor["name"]] = rospy.Publisher(
                    f'/rover_1/bmp280/{sensor["name"]}/temperature',
                    Float32,
                    queue_size=1,
                )
            except:
                print(f"Not found {sensor['name']}")

        for sensor in sensors_list["dht22"]:
            try:
                dht22_dict[sensor["name"]] = adafruit_dht.DHT22(
                    getattr(board, sensor["pin"])
                )

                dht22_temperature_publishers[sensor["name"]] = rospy.Publisher(
                    f'/rover_1/dht22/{sensor["name"]}/temperature',
                    Float32,
                    queue_size=1,
                )
                dht22_humidity_publishers[sensor["name"]] = rospy.Publisher(
                    f'/rover_1/dht22/{sensor["name"]}/humidity', Float32, queue_size=1
                )

            except:
                print(f"Not found {sensor['name']}")


def read_sensors():
    while not rospy.is_shutdown():
        for name, dht22 in dht22_dict.items():
            try:
                temperature = dht22.temperature
                humidity = dht22.humidity
                print(
                    f"{name} - Temperature: {temperature:.2f} - Humididty {humidity:.2f} %"
                )

                dht22_temperature_publishers[name].publish(Float32(temperature))
                dht22_humidity_publishers[name].publish(Float32(humidity))
            except:
                print(f"Not found DHT22 {name}")

        for name, bmp280 in bmp280_dict.items():
            try:
                temperature = bmp280.temperature
                print(f"{name} - Temperature: {temperature:.2f} C")
                bmp280_temperature_publishers[name].publish(Float32(temperature))
            except:
                print(f"Not found BMP280 {name}")

        rospy.sleep(2)


if __name__ == "__main__":
    try:
        rospy.init_node("temperature_sensors", anonymous=False)
        initialize_sensors()
        read_sensors()
    except rospy.ROSInterruptException:
        print("ROS node terminated.")
