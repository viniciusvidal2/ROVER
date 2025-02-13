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

endpoint_rest = "http://127.0.0.1:5000"
csv_file = 'temperature_data.csv'

global_dht22 = dict()
global_bmp280 = dict()
global_spi = board.SPI()
global_lantern = None

global_temperatures = dict()

# Init Flask app
app = Flask(__name__)
cors = CORS(app)

@app.route("/lantern/post", methods=["POST"])
def post_lantern():
    try:
        global global_lantern
        data = request.get_json()
        global_lantern.value = data["lantern"]
        return jsonify({"status": 1, "message": "Lantern get successfully"})
    except Exception as e:
        return jsonify({"status": 0, "error": str(e)}), 500
    
@app.route("/lantern/get", methods=["GET"])
def get_lantern():
    global global_lantern
    return jsonify({"lantern": int(global_lantern.value)})

@app.route("/temperatures/get", methods=["GET"])
def get_temperatures():
    global global_temperatures
    print(global_temperatures)
    return jsonify(global_temperatures)

def init_gpios(debug):
    global global_dht22, global_bmp280, global_spi, global_lantern
    
    with open('/home/rover/ROVER/gpios_control/gpios_pins.yaml', 'r') as file:
        gpios_list = yaml.safe_load(file)

        for gpio in gpios_list['bmp280']:
            try:
                bmp280_cs = DigitalInOut(getattr(board, gpio['cs_pin']))
                bmp280_cs.direction = Direction.OUTPUT
                bmp280_cs.value = 1
                bmp280 = adafruit_bmp280.Adafruit_BMP280_SPI(global_spi, bmp280_cs)
                global_bmp280[gpio['name']] = bmp280
            except Exception as e:
                global_bmp280[gpio['name']] = 0
                if debug:
                    print(f"Not found {gpio['name']} {e}")

        for gpio in gpios_list['dht22']:
            try:
                global_dht22[gpio['name']] = adafruit_dht.DHT22(
                    getattr(board, gpio['pin']))
            except Exception as e:
                global_dht22[gpio['name']] = 0
                if debug:
                    print(f"Not found {gpio['name']} {e}")
        
        try:            
            global_lantern = DigitalInOut(getattr(board, gpios_list['lantern'][0]["pin"]))
            global_lantern.direction = Direction.OUTPUT
        except Exception as e:
            if debug:
                print(f"Not found lantern {e}")

def main(debug: bool = False) -> None:
    """Reads the temperature and humidity from the sensors and sends the data to the rest api.

    Args:
        debug (bool, optional): Prints debug option. Defaults to False.
    """
    global global_dht22, global_bmp280, global_spi, global_lantern, global_temperatures
    
    init_gpios(debug)
    
    # Create CSV file with headers if it doesn't exist
    with open(csv_file, 'a', newline='') as f:
        if f.tell() == 0:  # File is empty
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'sensor', 'temperature', 'humidity'])
    
    # Main loop
    while True:
        timestamp = datetime.now().isoformat()
        # Open file in append mode for each write
        with open(csv_file, 'a', newline='') as f:
            writer = csv.writer(f)
            
            # Reading bmp280 sensors
            for name, bmp280 in global_bmp280.items():
                try:
                    temperature = bmp280.temperature
                    if debug:
                        print(f"{name} - Temperature: {temperature:.2f} C")
                    global_temperatures[name] = temperature
                    writer.writerow([timestamp, name, temperature, -1])
                except Exception as e:
                    global_temperatures[name] =  0
                    if debug:
                        print(f"Not found BMP280 {name} {e}")
                        
            # Reading dht22 sensors
            for name, dht22 in global_dht22.items():
                try:
                    temperature = dht22.temperature
                    humidity = dht22.humidity
                    if debug:
                        print(
                            f"{name} - Temperature: {temperature:.2f} - Humididty {humidity:.2f} %")
                    global_temperatures[name] = temperature
                    writer.writerow([timestamp, name, temperature, humidity])
                except Exception as e:
                    global_temperatures[name] = 0
                    if debug:
                        print(f"Not found DHT22 {name} {e}")
        
        # Sending POST request to the rest api
        # try:
        #     if global_temperatures:
        #         _ = requests.post(f"{endpoint_rest}/temperatures/post", json=global_temperatures)
        # except Exception as e:
        #     if debug:
        #         print(f"Error while making request: {e}")
        
        # We must wait 2 seconds before reading the sensors again
        time.sleep(2)


if __name__ == "__main__":
    main_thread = Thread(target=main, args=(False,), daemon=True)
    main_thread.start()
    
    app.run(host="0.0.0.0", port=5001)
