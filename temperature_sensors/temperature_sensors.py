import time
import board
from digitalio import DigitalInOut, Direction
import adafruit_dht
import adafruit_bmp280
import yaml
import requests
import csv
from datetime import datetime


def main(debug: bool = False) -> None:
    """Reads the temperature and humidity from the sensors and sends the data to the rest api.

    Args:
        debug (bool, optional): Prints debug option. Defaults to False.
    """
    # Rest API endpoint to send the temperature data
    endpoint = "http://127.0.0.1:5000/temperatures/post"
    csv_file = 'temperature_data.csv'

    # Create CSV file with headers if it doesn't exist
    with open(csv_file, 'a', newline='') as f:
        if f.tell() == 0:  # File is empty
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'sensor', 'temperature', 'humidity'])

    # Sensors data
    dht22_dict = {}
    bmp280_dict = {}
    spi = board.SPI()

    # Opening the sensors from the list in the yaml file
    # Sensors setup process
    with open('sensors.yaml', 'r') as file:
        sensors_list = yaml.safe_load(file)

        for sensor in sensors_list['bmp280']:
            try:
                bmp280_cs = DigitalInOut(getattr(board, sensor['cs_pin']))
                bmp280_cs.direction = Direction.OUTPUT
                bmp280_cs.value = 1
                bmp280 = adafruit_bmp280.Adafruit_BMP280_SPI(spi, bmp280_cs)
                bmp280_dict[sensor['name']] = bmp280
            except Exception as e:
                if debug:
                    print(f"Not found {sensor['name']} {e}")

        for sensor in sensors_list['dht22']:
            try:
                dht22_dict[sensor['name']] = adafruit_dht.DHT22(
                    getattr(board, sensor['pin']))
            except Exception as e:
                if debug:
                    print(f"Not found {sensor['name']} {e}")

    # Main loop
    while True:
        timestamp = datetime.now().isoformat()
        sensors_data = {}
        # Open file in append mode for each write
        with open(csv_file, 'a', newline='') as f:
            writer = csv.writer(f)

            # Reading dht sensors
            for name, dht22 in dht22_dict.items():
                try:
                    temperature = dht22.temperature
                    humidity = dht22.humidity
                    if debug:
                        print(
                            f"{name} - Temperature: {temperature:.2f} - Humididty {humidity:.2f} %")
                    sensors_data[name] = {
                        "temperature": temperature, "humidity": humidity}
                    writer.writerow([timestamp, name, temperature, humidity])
                except Exception as e:
                    if debug:
                        print(f"Not found DHT22 {name} {e}")
            # Reading bmp sensors
            for name, bmp280 in bmp280_dict.items():
                try:
                    temperature = bmp280.temperature
                    if debug:
                        print(f"{name} - Temperature: {temperature:.2f} C")
                    sensors_data[name] = {"temperature": temperature}
                    writer.writerow([timestamp, name, temperature, -1])
                except Exception as e:
                    if debug:
                        print(f"Not found BMP280 {name} {e}")
        # Sending POST request to the rest api
        try:
            if sensors_data:
                _ = requests.post(endpoint, json=sensors_data)
        except Exception as e:
            if debug:
                print(f"Error while making request: {e}")
        # We must wait 2 seconds before reading the sensors again
        time.sleep(2)


if __name__ == "__main__":
    main(debug=True)
