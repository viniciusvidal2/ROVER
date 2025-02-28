import os
from time import time
from map_manager import MapManager


if __name__ == "__main__":
    start = time()
    # Get the folder containing all the map data
    folder = os.path.join(os.getenv("HOME"), "maps")
    # Create the map manager object for the specific map
    map_name = "map_1"
    map_folder = os.path.join(folder, map_name)
    desired_map_size = [1080, 1920]  # [lines, collumns]
    robot_map_manager = MapManager(
        map_name=map_name, folder=map_folder, imsize=desired_map_size)
    # Call the point cloud and bev generations
    map_ptc = robot_map_manager.generate_map_ptc()
    world_T_map = robot_map_manager.compute_world_T_map()
    map_bev, map_coords_json = robot_map_manager.generate_map_bev(
        ptc_map_frame=map_ptc, world_T_map=world_T_map)
    # Save the files
    robot_map_manager.save_map(
        map_bev=map_bev, map_ptc=map_ptc, map_coords=map_coords_json, world_T_map=world_T_map)
    end = time()
    print(f"Time taken: {end - start} s")
