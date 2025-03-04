# -*- coding: utf-8 -*-
from flask import Flask, jsonify, request
from flask_cors import CORS
import roslibpy
from sensor_msgs.msg import CompressedImage, NavSatFix
from std_msgs.msg import Float64
from time import sleep
import subprocess
from threading import Thread
from time import sleep, time
from mqtt_handler import MqttHandler
import requests
import os
import shutil
import re

############################################################################
# region Declarations and Definitions
############################################################################

# Connection HTTP to GPIOs
endpoint_gpios = "http://127.0.0.1:5001"

# Connection MQTT to Dashboard
global_mqtt = None
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
# SUB001 e Rover-Argo-N-0 vão ser variáveis
MQTT_TOPIC = "substations/SUB001/rovers/Rover-Argo-N-0"
TELEMETRY_INTERVAL = 2

# Connection to ROS
global_ros_ip = "localhost"
global_ros_port = 9090
ros = roslibpy.Ros(host=global_ros_ip, port=global_ros_port)

# Global variables and topics
global_image_camera = None
global_gps_coordinates = {"latitude": -1, "longitude": -1}
global_compass_heading = -1  # -1 means no data [degrees 0-360]
global_image_topic = roslibpy.Topic(
    ros, "/image_user/compressed", "sensor_msgs/CompressedImage"
)
global_gps_topic = roslibpy.Topic(
    ros, "/mavros/global_position/global", "sensor_msgs/NavSatFix"
)
global_compass_topic = roslibpy.Topic(
    ros, "/mavros/global_position/compass_hdg", "std_msgs/Float64"
)
global_status_text_topic = roslibpy.Topic(
    ros, "/mavros/statustext/send", "mavros_msgs/Statustext"
)


def create_destination_folder(folder: str) -> str:
    """Certifies that the name does not exist, and if it does adapt it not to have the same name

    Args:
        folder (str): original folder name

    Returns:
        str: new folder name
    """
    i = 1
    while os.path.exists(folder):
        folder = f"{folder}_{i}"
        i += 1
    return folder


def clear_folder(folder_path: str) -> None:
    """Clears the folder contents, but not the folder itself

    Args:
        folder_path (str): path to the folder
    """
    for item in os.listdir(folder_path):
        item_path = os.path.join(folder_path, item)
        if os.path.isfile(item_path) or os.path.islink(item_path):
            os.unlink(item_path)  # Remove file or symbolic link
        elif os.path.isdir(item_path):
            shutil.rmtree(item_path)  # Remove directory and its contents


# Init Flask app
app = Flask(__name__)
cors = CORS(app)

# endregion
############################################################################
# region API POSTs
############################################################################


@app.route("/mapping/start", methods=["POST"])
def start_mapping():
    # Kill existing process if running
    subprocess.run(["rosnode", "kill", "/mapping_node", "/lidar_odometry_node", "/preprocess_lidar_scan_node"],
                   stderr=subprocess.DEVNULL)
    # Start new mapping process
    try:
        data = request.get_json()
        map_name = "map_1"
        if data:
            map_name = data["map_name"]
        subprocess.Popen(
            [
                "roslaunch",
                "mapping",
                "mapping.launch",
                f"map_name:={map_name}",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        return jsonify({"status": 1, "message": "Mapping launched successfully"})
    except Exception as e:
        return jsonify({"status": 0, "error": str(e)}), 500


@app.route("/mapping/stop", methods=["POST"])
def stop_mapping():
    try:
        subprocess.Popen(
            ["rosnode", "kill", "/mapping_node",
                "/lidar_odometry_node", "/preprocess_lidar_scan_node"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        return jsonify({"status": 1, "message": "Mapping node stopped successfully"})
    except Exception as e:
        return jsonify({"status": 0, "error": str(e)}), 500


@app.route("/localization/start", methods=["POST"])
def start_localization():
    # Kill existing process if running
    subprocess.run(["rosnode", "kill", "/localization_node", "/lidar_odometry_node", "/preprocess_lidar_scan_node", "/obstacle_generator_node"],
                   stderr=subprocess.DEVNULL)
    # Start new localization process
    try:
        data = request.get_json()
        data = request.get_json()
        map_name = "map_1"
        if data:
            map_name = data["map_name"]
        subprocess.Popen(
            [
                "roslaunch",
                "localization",
                "localization.launch",
                f"map_name:={map_name}",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        return jsonify({"status": 1, "message": "Localization launched successfully"})
    except Exception as e:
        return jsonify({"status": 0, "error": str(e)}), 500


@app.route("/localization/stop", methods=["POST"])
def stop_localization():
    try:
        subprocess.Popen(
            ["rosnode", "kill", "/localization_node", "/lidar_odometry_node",
                "/preprocess_lidar_scan_node", "/obstacle_generator_node"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        return jsonify(
            {"status": 1, "message": "Localization node stopped successfully"}
        )
    except Exception as e:
        return jsonify({"status": 0, "error": str(e)}), 500


@app.route("/status_text", methods=["POST"])
def publish_status_text():
    try:
        # Sending status text with mavros for the rover to be relayed to the radio controller
        data = request.get_json()
        global_status_text_topic.publish(
            roslibpy.Message(
                {"severity": data["severity"], "text": data["text"]})
        )
        return jsonify({"status": 1, "message": "Status text published successfully"})
    except Exception as e:
        return jsonify({"status": 0, "error": str(e)}), 500


@app.route("/system/start_bag_record", methods=["POST"])
def start_bag_record():
    # Kill existing rosbag process if running
    subprocess.run(["rosnode", "kill", "/rosbag_recorder"],
                   stderr=subprocess.DEVNULL)
    # Start new rosbag process
    topics = ["/livox/lidar", "/livox/imu", "/mavros/imu/data", "/mavros/setpoint_raw/target_global", "/mavros/state",
              "/mavros/global_position/global", "/mavros/global_position/compass_hdg", "/mavros/mission/waypoints", "/mavros/home_position/home"]
    duration = "30"
    save_dir = "/home/rover/bags_debug"
    file_names = os.path.join(save_dir, "rosbag_debug")
    try:
        data = request.get_json()
        file_prefix = "rosbag_debug"
        if data:
            file_prefix = data["bag_name"]
        file_names = os.path.join(save_dir, file_prefix)
        command = ["rosbag", "record", "-o", file_names, "--split",
                   "--duration", duration, "__name:=rosbag_recorder"]
        command[2:2] = topics
        subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        return jsonify({"status": 1, "message": "Bag record launched successfully"})
    except Exception as e:
        return jsonify({"status": 0, "error": str(e)}), 500


@app.route("/system/stop_bag_record", methods=["POST"])
def stop_bag_record():
    try:
        subprocess.Popen(
            ["rosnode", "kill", "/rosbag_recorder"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        return jsonify({"status": 1, "message": "Bag record stopped successfully"})
    except Exception as e:
        return jsonify({"status": 0, "error": str(e)}), 500


@app.route("/system/remove_map", methods=["POST"])
def remove_map():
    try:
        data = request.get_json()
        map_name = data["map_name"]
        shutil.rmtree(f"/root/maps/{map_name}")
        return jsonify({"status": 1, "message": "Map removed successfully"})
    except Exception as e:
        return jsonify({"status": 0, "error": str(e)}), 500


@app.route("/system/copy_data_usb", methods=["POST"])
def copy_data_usb():
    MEDIA_FOLDER = "/home/rover/media"
    MAPS_FOLDER = "/root/maps"
    BAGS_FOLDER = "/home/rover/bags_debug"
    # List the usb devices in MEDIA FOLDER
    devices = os.listdir(MEDIA_FOLDER)
    if len(devices) == 0:
        return jsonify({"status": 0, "error": "No USB devices found"}), 500
    # Copy the data to all the devices
    try:
        for device in devices:
            destination_maps_folder = create_destination_folder(
                f"{MEDIA_FOLDER}/{device}/maps")
            destination_bags_folder = create_destination_folder(
                f"{MEDIA_FOLDER}/{device}/bags_debug")
            shutil.copytree(MAPS_FOLDER, destination_maps_folder)
            shutil.copytree(BAGS_FOLDER, destination_bags_folder)
        # Remove the content in the maps and bags_debug folders
        # We must not delete the folders
        clear_folder(MAPS_FOLDER)
        clear_folder(BAGS_FOLDER)
        return jsonify({"status": 1, "message": "Data transfered to USB successfully"})
    except Exception as e:
        return jsonify({"status": 0, "error": str(e)}), 500

# endregion
############################################################################
# region API GETs
############################################################################


@app.route("/insta360/image", methods=["GET"])
def get_camera_image() -> dict:
    global global_image_camera
    return global_image_camera


@app.route("/gps/location_orientation", methods=["GET"])
def get_gps_location_orientation() -> dict:
    global global_gps_coordinates, global_compass_heading
    # Check if we have data (1), if not return status = 0
    status = 1
    if (
        global_gps_coordinates["latitude"] == -1
        or global_gps_coordinates["longitude"] == -1
        or global_compass_heading == -1
    ):
        status = 0

    return jsonify(
        {
            "status": status,
            "latitude": global_gps_coordinates["latitude"],
            "longitude": global_gps_coordinates["longitude"],
            "compass": global_compass_heading,
        }
    )


@app.route("/system/device_space", methods=["GET"])
def get_device_space() -> dict:
    try:
        # Get how much of the space is still free, in GB
        data = subprocess.check_output(["df", "-h"], text=True)
        lines = data.split("\n")
        for line in lines:
            space, section = line.split()[3], line.split()[5]
            if section == "/":
                break
        match = re.match(r"(\d+\.?\d*)([A-Za-z])", space)
        if match:
            number = float(match.group(1))
            unit = match.group(2)
        BAG_MB_PER_SEC = 3.46  # [MB/s]
        bag_minutes_left = number * 1024 / BAG_MB_PER_SEC / 60
        if unit != "G":
            bag_minutes_left /= 1024
        return jsonify({"status": 1, "number": number, "unit": unit, "bag_minutes_left": bag_minutes_left})
    except Exception as e:
        return jsonify({"status": 0, "error": str(e)}), 500


@app.route("/system/maps_done", methods=["GET"])
def get_maps_done() -> dict:
    try:
        # Get the maps that are already done
        maps = sorted(os.listdir("/root/maps"))
        return jsonify({"status": 1, "maps": maps})
    except Exception as e:
        return jsonify({"status": 0, "error": str(e)}), 500


@app.route("/system/mapping_status", methods=["GET"])
def get_mapping_status() -> dict:
    try:
        # Get the status of the mapping node
        data = subprocess.check_output(["rosnode", "list"], text=True)
        if "/mapping_node" in data:
            return jsonify({"status": 1, "mapping_status": 1})
        else:
            return jsonify({"status": 1, "mapping_status": 0})
    except Exception as e:
        return jsonify({"status": 0, "error": str(e)}), 500


@app.route("/system/rosbag_status", methods=["GET"])
def get_rosbag_status() -> dict:
    try:
        # Get the status of the mapping node
        data = subprocess.check_output(["rosnode", "list"], text=True)
        if "/rosbag_recorder" in data:
            return jsonify({"status": 1, "rosbag_status": 1})
        else:
            return jsonify({"status": 1, "rosbag_status": 0})
    except Exception as e:
        return jsonify({"status": 0, "error": str(e)}), 500

# endregion
############################################################################
# region MQTT Functions
############################################################################


def publish_telemetry():
    # TODO speed, status
    global global_mqtt, global_gps_coordinates, global_compass_heading
    try:
        lantern = requests.get(f"{endpoint_gpios}/lantern/get").json()
    except Exception as e:
        lantern = {"lantern": -1}
        print(f"Error get lantern {e}")
    try:
        temperatures = requests.get(
            f"{endpoint_gpios}/temperatures/get"
        ).json()
    except Exception as e:
        temperatures = {"T": -1}
        print(f"Error get temperatures {e}")
    try:
        battery = requests.get(f"{endpoint_gpios}/soc/get").json()["soc"]
    except Exception as e:
        battery = -1
        print(f"Error get battery SOC {e}")
    global_mqtt.publish_telemetry(
        temperatures,
        battery,
        12,
        global_gps_coordinates,
        global_compass_heading,
        "active",
        lantern,
    )


def publishers() -> None:
    global global_mqtt, global_gps_coordinates, global_compass_heading

    last_telemetry = 0

    while True:
        try:
            current_time = time()

            if current_time - last_telemetry >= TELEMETRY_INTERVAL:
                publish_telemetry()
                last_telemetry = current_time

            if global_mqtt.flag_gps:
                global_mqtt.publish_gps(
                    global_gps_coordinates, global_compass_heading)
                global_mqtt.flag_gps = 0

            if global_mqtt.flag_lantern != -1:
                lantern_data = {"lantern": global_mqtt.flag_lantern}
                _ = requests.post(
                    f"{endpoint_gpios}/lantern/post", json=lantern_data)
                global_mqtt.flag_lantern = -1

            if global_mqtt.flag_bms:
                bms = requests.get(f"{endpoint_gpios}/bms/get").json()
                global_mqtt.publish_bms(bms)
                global_mqtt.flag_bms = 0

            sleep(0.01)
        except Exception as e:
            print(f"Error in MQTT thread: {e}")
            sleep(10)


# endregion
############################################################################
# region ROS data callbacks
############################################################################


def camera_image_callback(msg: CompressedImage) -> None:
    global global_image_camera
    global_image_camera = msg["data"]


def gps_callback(msg: NavSatFix) -> None:
    global global_gps_coordinates
    global_gps_coordinates = {
        "latitude": msg["latitude"],
        "longitude": msg["longitude"],
    }


def compass_callback(msg: Float64) -> None:
    # The compass sends data in DEGREES (0 - 360)
    # It has North as 0, East as 90, South as 180 and West as 270
    global global_compass_heading
    global_compass_heading = msg["data"]  # [degrees]


# endregion
############################################################################
# region Main functions
############################################################################


def guarantee_ros_connection() -> None:
    # Check if rosbridge is already connected
    rosbridge_up = False
    while not rosbridge_up:
        try:
            output = subprocess.check_output(["rosnode", "list"], text=True)
            if "/rosbridge_websocket" in output:
                rosbridge_up = True
        except subprocess.CalledProcessError:
            print("Failed to check rosbridge status.")

    # Start the ros connection
    ros.run()
    while not ros.is_connected:
        print("Is ROS connected ?", ros.is_connected)
        ros.connect()
        sleep(1)
    print("ROS connected")
    ros.run()


def init_subscribers() -> None:
    global global_image_topic, global_gps_topic, global_compass_topic

    global_image_topic.subscribe(callback=camera_image_callback)
    global_gps_topic.subscribe(callback=gps_callback)
    global_compass_topic.subscribe(callback=compass_callback)


if __name__ == "__main__":
    # Connect to ROS
    guarantee_ros_connection()
    # Init all ROS subscribers
    init_subscribers()
    # Init MQTT client
    global_mqtt = MqttHandler(MQTT_BROKER, MQTT_PORT, MQTT_TOPIC)
    # Start MQTT publishing thread
    mqtt_thread = Thread(target=publishers, daemon=True)
    mqtt_thread.start()
    # Run the app to serve the API
    app.run(host="0.0.0.0", port=5000)

    # Disconnect from MQTT and Servos
    global_mqtt.close()
    # Disconnect from ROS
    ros.close()

# endregion
