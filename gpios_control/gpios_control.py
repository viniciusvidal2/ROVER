import time
import board
import digitalio
from digitalio import DigitalInOut, Direction
import adafruit_dht
import adafruit_bmp280
import yaml
import requests
import csv
from datetime import datetime
from flask import Flask, jsonify, request
from flask_cors import CORS
from threading import Thread
from dalybms import DalyBMS
import psutil

############################################################################
# region Declarations and Definitions
############################################################################

# CSV file to store temperature data
csv_file = "temperature_data.csv"

# GPIOs objects
global_dht22 = dict()
global_bmp280 = dict()
global_spi = board.SPI()
global_lantern = None
global_bms = DalyBMS(address=4)
BMS_USB = "/dev/ttyAMA0"

# data variables
global_temperatures = dict()

# Init Flask app
app = Flask(__name__)
cors = CORS(app)

# endregion

############################################################################
# region GPIOs POSTs
############################################################################


@app.route("/lantern/post", methods=["POST"])
def post_lantern():
    try:
        global global_lantern
        data = request.get_json()
        global_lantern.value = data["lantern"]
        return jsonify({"status": 1, "message": "Lantern get successfully"})
    except Exception as e:
        return jsonify({"status": 0, "error": str(e)}), 500


# endregion

############################################################################
# region GPIOs GETs
############################################################################


@app.route("/lantern/get", methods=["GET"])
def get_lantern():
    global global_lantern
    try:
        return jsonify({"lantern": int(global_lantern.value)})
    except Exception as e:
        return jsonify({"lantern": -1})


@app.route("/temperatures/get", methods=["GET"])
def get_temperatures():
    global global_temperatures
    try:
        global_temperatures = global_bms.get_temperatures()
        global_temperatures = {f"T{key}": value for key,
                               value in global_temperatures.items()}
        global_temperatures["TRPI"] = psutil.sensors_temperatures()[
            'cpu_thermal'][0].current
        return jsonify(global_temperatures)
    except Exception as e:
        print(f"Error temperatures BMS {e}")
        return jsonify({"T": -1})


@app.route("/soc/get", methods=["GET"])
def get_soc():
    global global_bms
    try:
        soc = global_bms.get_soc()
        return jsonify({"soc": soc["soc_percent"]})
    except Exception as e:
        print(f"Error SOC BMS {e}")
        return jsonify({"soc": -1})


@app.route("/bms/get", methods=["GET"])
def get_bms():
    global global_bms
    try:
        data = global_bms.get_all()
        return jsonify(data)
    except Exception as e:
        print(f"Error all BMS {e}")
        return jsonify({"bms": -1})


# endregion

############################################################################
# region Init Pins
############################################################################


def init_gpios(debug):
    global global_dht22, global_bmp280, global_spi, global_lantern, global_bms

    try:
        global_bms.connect(BMS_USB)
    except Exception as e:
        print(f"Error init BMS {e}")

    with open("/home/rover/ROVER/gpios_control/gpios_pins.yaml", "r") as file:
        gpios_list = yaml.safe_load(file)

        # for gpio in gpios_list["bmp280"]:
        #     try:
        #         bmp280_cs = DigitalInOut(getattr(board, gpio["pin"]))
        #         bmp280_cs.direction = Direction.OUTPUT
        #         bmp280_cs.value = 1
        #         bmp280 = adafruit_bmp280.Adafruit_BMP280_SPI(global_spi, bmp280_cs)
        #         global_bmp280[gpio["name"]] = bmp280
        #     except Exception as e:
        #         global_bmp280[gpio["name"]] = None
        #         if debug:
        #             print(f"Not found {gpio['name']} {e}")

        # for gpio in gpios_list["dht22"]:
        #     try:
        #         global_dht22[gpio["name"]] = adafruit_dht.DHT22(
        #             getattr(board, gpio["pin"])
        #         )
        #     except Exception as e:
        #         global_dht22[gpio["name"]] = None
        #         if debug:
        #             print(f"Not found {gpio['name']} {e}")

        try:
            global_lantern = DigitalInOut(
                getattr(board, gpios_list["lantern"][0]["pin"])
            )
            global_lantern.direction = Direction.OUTPUT
        except Exception as e:
            if debug:
                print(f"Not found lantern {e}")


# endregion


############################################################################
# region Main
############################################################################


def main(debug: bool = False) -> None:
    """Reads the temperature and humidity from the sensors and sends the data to the rest api.

    Args:
        debug (bool, optional): Prints debug option. Defaults to False.
    """
    global global_dht22, global_bmp280, global_spi, global_lantern, global_temperatures, global_bms

    init_gpios(debug)

    # Create CSV file with headers if it doesn't exist
    # with open(csv_file, "a", newline="") as f:
    #     if f.tell() == 0:  # File is empty
    #         writer = csv.writer(f)
    #         writer.writerow(["timestamp", "sensor", "temperature", "humidity"])

    # Main loop
    # while True:
    #     try:
    #         timestamp = datetime.now().isoformat()
    #         # Open file in append mode for each write
    #         with open(csv_file, "a", newline="") as f:
    #             writer = csv.writer(f)

    #             # Reading bmp280 sensors
    #             for name, bmp280 in global_bmp280.items():
    #                 try:
    #                     temperature = bmp280.temperature
    #                     if debug:
    #                         print(f"{name} - Temperature: {temperature:.2f} C")
    #                     global_temperatures[name] = temperature
    #                     writer.writerow([timestamp, name, temperature, -1])
    #                 except Exception as e:
    #                     global_temperatures[name] = 0
    #                     if debug:
    #                         print(f"Not found BMP280 {name} {e}")

    #             # Reading dht22 sensors
    #             for name, dht22 in global_dht22.items():
    #                 try:
    #                     temperature = dht22.temperature
    #                     humidity = dht22.humidity
    #                     if debug:
    #                         print(
    #                             f"{name} - Temperature: {temperature:.2f} - Humididty {humidity:.2f} %"
    #                         )
    #                     global_temperatures[name] = temperature
    #                     writer.writerow([timestamp, name, temperature, humidity])
    #                 except Exception as e:
    #                     global_temperatures[name] = 0
    #                     if debug:
    #                         print(f"Not found DHT22 {name} {e}")

    #         # We must wait 2 seconds before reading the sensors again
    #         time.sleep(2)
    #     except Exception as e:
    #         print(f"Error in Sensors thread: {e}")
    #         time.sleep(10)


# endregion
if __name__ == "__main__":
    # main_thread = Thread(target=main, args=(False,), daemon=True)
    # main_thread.start()
    init_gpios(False)
    app.run(host="0.0.0.0", port=5001)

    # When exiting the script, close connection properly
    if hasattr(global_bms, "serial") and global_bms.serial.is_open:
        global_bms.disconnect()
