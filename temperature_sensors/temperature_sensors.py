import time
import board
from digitalio import DigitalInOut, Direction
import adafruit_dht
import adafruit_bmp280
import yaml
import requests

endpoint = '"http://localhost:5000/temperatures/post'

dht22_dict = {}

spi = board.SPI()
bmp280_dict = {}

with open('sensors.yaml', 'r') as file:
    sensors_list = yaml.safe_load(file)
    
    for sensor in sensors_list['bmp280']:
        try:
            bmp280_cs = DigitalInOut(getattr(board, sensor['cs_pin']))
            bmp280_cs.direction = Direction.OUTPUT
            bmp280_cs.value = 1
            bmp280 = adafruit_bmp280.Adafruit_BMP280_SPI(spi, bmp280_cs)
            bmp280_dict[sensor['name']] = bmp280
        except:
            print(f"Not found {sensor['name']}")
            
    for sensor in sensors_list['dht22']:
        try:
            dht22_dict[sensor['name']] = adafruit_dht.DHT22(getattr(board, sensor['pin']))
        except:
            print(f"Not found {sensor['name']}")

while True:
    sensors_data = {}
    
    for name, dht22 in dht22_dict.items():
        try:
            temperature = dht22.temperature
            humidity = dht22.humidity
            print(f"{name} - Temperature: {temperature:.2f} - Humididty {humidity:.2f} %")
            sensors_data[name] = {"temperature": temperature, "humidity": humidity}
        except:
            print(f"Not found DHT22 {name}")

    for name, bmp280 in bmp280_dict.items():
        try:
            temperature = bmp280.temperature
            print(f"{name} - Temperature: {temperature:.2f} C")
            sensors_data[name] = {"temperature": temperature}
        except:
            print(f"Not found BMP280 {name}")
            
    try:
        if sensors_data:
            _ = requests.post(endpoint, json=sensors_data)
    except Exception as e:
        print(f"Error while making request: {e}")

    time.sleep(2)
