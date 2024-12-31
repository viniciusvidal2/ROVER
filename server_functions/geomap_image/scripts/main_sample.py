import os
from time import time
from map_manager import MapManager


if __name__ == "__main__":
    start = time()
    # Get the folder containing the map data
    folder = os.path.join(os.getenv("HOME"), "maps/map_1")
    # Create the map manager object for the specific map robot
    rover_code = 0
    map_folder = folder
    desired_map_size = [1080, 1920]  # [lines, collumns]
    robot_map_manager = MapManager(
        rover_code=rover_code, folder=map_folder, imsize=desired_map_size)
    # Call the point cloud and bev generations
    map_ptc = robot_map_manager.generateMapPtc()
    map_bev, map_coords_json = robot_map_manager.generateMapBev(
        ptc_map_frame=map_ptc)
    # Save the files
    robot_map_manager.saveMap(
        map_bev=map_bev, map_ptc=map_ptc, map_coords=map_coords_json)
    end = time()
    print(f"Time taken: {end - start} s")
