import utm
import os
import msgpack
import cv2

if __name__ == "__main__":
    # Get the folder containing the map data
    folder = os.path.join(os.getenv("HOME"), "maps/map_1")
    # Rover name to access files based on its code
    rover_code = 0
    rover_name = f"rover_{rover_code}"
    # Read the georef file compressed with msgpack
    georef_file = os.path.join(folder, f"{rover_name}.msgpack")
    with open(georef_file, "rb") as f:
        georef_data = msgpack.unpackb(f.read())
    # Read the bev image as grayscale
    bev_file = os.path.join(folder, f"{rover_name}.png")
    bev = cv2.imread(bev_file, cv2.IMREAD_GRAYSCALE)
    # Get the image size and use the center of the image to create a key
    img_size = bev.shape[:2]
    key = f"{img_size[0]//2}_{img_size[1]//2}"
    # Get the georef data for the key
    georef = georef_data[key]
    # Get the latitude and longitude of the center of the image
    lat, lon = utm.to_latlon(georef["utm_e"], georef["utm_n"], 23, "k")
    print(f"UTM data: {georef}")
    print(f"Latitude: {lat}, Longitude: {lon}")
    # Display the bev image
    cv2.imshow("BEV", bev)
    cv2.waitKey(0)
