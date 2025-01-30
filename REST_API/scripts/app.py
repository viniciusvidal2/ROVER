# -*- coding: utf-8 -*-
from flask import Flask, jsonify, request
from flask_cors import CORS
import roslibpy
from sensor_msgs.msg import CompressedImage, NavSatFix
from std_msgs.msg import Float64
from time import sleep
import subprocess

############################################################################
# region Declarations and Definitions
############################################################################

# Connection to ROS
global_ros_ip = 'localhost'
global_ros_port = 9090
ros = roslibpy.Ros(host=global_ros_ip, port=global_ros_port)

# Global variables and topics
global_image_camera = None
global_gps_coordinates = {'latitude': -1, 'longitude': -1}
global_compass_heading = -1  # -1 means no data [degrees 0-360]
global_temperatures = dict()
global_image_topic = roslibpy.Topic(
    ros, '/image_user/compressed', 'sensor_msgs/CompressedImage')
global_gps_topic = roslibpy.Topic(
    ros, '/mavros/global_position/global', 'sensor_msgs/NavSatFix')
global_compass_topic = roslibpy.Topic(
    ros, '/mavros/global_position/compass_hdg', 'std_msgs/Float64')

# Init Flask app
app = Flask(__name__)
cors = CORS(app)

# endregion
############################################################################
# region API POSTs
############################################################################


@app.route('/mapping/start', methods=['POST'])
def start_mapping():
    try:
        data = request.get_json()
        if data is None:
            subprocess.Popen(['roslaunch', 'mapping', 'mapping.launch'],
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        else:
            subprocess.Popen(['roslaunch', 'mapping', 'mapping.launch', 'map_name:=' + data['map_name']],
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        return jsonify({"status": 1, "message": "Mapping launched successfully"})
    except Exception as e:
        return jsonify({"status": 0, "error": str(e)}), 500


@app.route('/mapping/stop', methods=['POST'])
def stop_mapping():
    try:
        subprocess.Popen(['rosnode', 'kill', '/mapping_node', '/lidar_odometry_node'],
                         stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        return jsonify({"status": 0, "message": "Mapping node stopped successfully"})
    except Exception as e:
        return jsonify({"status": 1, "error": str(e)}), 500


@app.route('/localization/start', methods=['POST'])
def start_localization():
    try:
        data = request.get_json()
        if data is None:
            subprocess.Popen(['roslaunch', 'localization', 'localization.launch'],
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        else:
            subprocess.Popen(['roslaunch', 'localization', 'localization.launch', 'map_name:=' + data['map_name']],
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        return jsonify({"status": 1, "message": "Localization launched successfully"})
    except Exception as e:
        return jsonify({"status": 0, "error": str(e)}), 500


@app.route('/localization/stop', methods=['POST'])
def stop_localization():
    try:
        subprocess.Popen(['rosnode', 'kill', '/localization_node', '/lidar_odometry_node'],
                         stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        return jsonify({"status": 0, "message": "Localization node stopped successfully"})
    except Exception as e:
        return jsonify({"status": 1, "error": str(e)}), 500


@app.route('/temperatures/post', methods=['POST'])
def post_temperatures():
    try:
        global global_temperatures
        data = request.get_json()
        if data:
            global_temperatures = data
        return jsonify({"status": 1, "message": "Temperatures get successfully"})
    except Exception as e:
        return jsonify({"status": 0, "error": str(e)}), 500

# endregion
############################################################################
# region API GETs
############################################################################


@app.route('/insta360/image', methods=['GET'])
def get_camera_image() -> dict:
    global global_image_camera
    return global_image_camera


@app.route('/gps/location_orientation', methods=['GET'])
def get_gps_location_orientation() -> dict:
    global global_gps_coordinates, global_compass_heading
    # Check if we have data (1), if not return status = 0
    status = 1
    if global_gps_coordinates['latitude'] == -1 or global_gps_coordinates['longitude'] == -1 or global_compass_heading == -1:
        status = 0

    return jsonify({'status': status,
                    'latitude': global_gps_coordinates['latitude'],
                    'longitude': global_gps_coordinates['longitude'],
                    'compass': global_compass_heading})


@app.route('/temperatures/get', methods=['GET'])
def get_temperatures() -> dict:
    global global_temperatures
    return jsonify(global_temperatures)

# endregion
############################################################################
# region ROS data callbacks
############################################################################


def camera_image_callback(msg: CompressedImage) -> None:
    global global_image_camera
    global_image_camera = msg['data']


def gps_callback(msg: NavSatFix) -> None:
    global global_gps_coordinates
    global_gps_coordinates = {
        'latitude': msg['latitude'], 'longitude': msg['longitude']}


def compass_callback(msg: Float64) -> None:
    # The compass sends data in DEGREES (0 - 360)
    # It has North as 0, East as 90, South as 180 and West as 270
    global global_compass_heading
    global_compass_heading = msg['data']  # [degrees]


# endregion
############################################################################
# region Main functions
############################################################################

def guarantee_ros_connection() -> None:
    # Check if rosbridge is already connected
    rosbridge_up = False
    while not rosbridge_up:
        try:
            output = subprocess.check_output(['rosnode', 'list'], text=True)
            if '/rosbridge_websocket' in output:
                rosbridge_up = True
        except subprocess.CalledProcessError:
            print("Failed to check rosbridge status.")
    # Start the ros connection
    ros.run()
    while not ros.is_connected:
        print('Is ROS connected ?', ros.is_connected)
        ros.connect()
        sleep(1)
    print('ROS connected')
    ros.run()


def init_subscribers() -> None:
    global global_image_topic, global_gps_topic, global_compass_topic

    global_image_topic.subscribe(callback=camera_image_callback)
    global_gps_topic.subscribe(callback=gps_callback)
    global_compass_topic.subscribe(callback=compass_callback)


if __name__ == '__main__':
    # Connect to ROS
    guarantee_ros_connection()
    # Init all ROS subscribers
    init_subscribers()
    # Run the app to serve the API
    app.run(host='0.0.0.0', port=5000)

    # Disconnect from ROS
    ros.close()


# endregion
