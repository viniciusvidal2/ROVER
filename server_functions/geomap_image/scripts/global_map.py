import os
import numpy as np
import open3d as o3d
from map_manager import MapManager
from copy import deepcopy


def main(maps_folder: "str") -> None:
    """Creates the global map from the maps in the maps folder

    Args:
        maps_folder (str): the maps folder path
    """
    # Global data variables
    global_map_point_cloud = o3d.geometry.PointCloud()
    maps_folder = os.path.join(os.getenv("HOME"), "maps")
    world_T_map_ref = None  # We should initialize with the first reading

    # Manage the folders to allocate the global map properly
    global_map_name = "global_map"
    global_map_folder = os.path.join(maps_folder, global_map_name)
    if not os.path.exists(global_map_folder):
        os.makedirs(global_map_folder)
    # Try to read the world_T_map_ref from the global map folder
    world_T_map_ref_path = os.path.join(
        global_map_folder, global_map_name + ".npy")
    if os.path.exists(world_T_map_ref_path):
        world_T_map_ref = np.load(world_T_map_ref_path)

    # Iterate over all the folders in the maps folder
    for map_name in sorted(os.listdir(maps_folder)):
        map_folder = os.path.join(maps_folder, map_name)
        # If the file is not a folder or the global map folder, skip it
        if not os.path.isdir(map_folder) or map_name == global_map_name:
            continue

        # If the folder already contains a map point cloud, just load it with the transformation
        map_ptc = o3d.geometry.PointCloud()
        world_T_map = np.eye(4)
        ptc_path = os.path.join(map_folder, map_name + ".pcd")
        transform_path = os.path.join(map_folder, map_name + ".npy")
        if os.path.exists(ptc_path):
            print(f"Loading map point cloud for {map_folder}")
            map_ptc = o3d.io.read_point_cloud(ptc_path)
            world_T_map = np.load(transform_path)
        # If not, we must create everything with the maps manager
        else:
            print(f"Creating map point cloud for {map_folder}")
            # Create the map manager object for the specific map
            desired_map_size = [1080, 1920]  # [lines, collumns]
            robot_map_manager = MapManager(
                map_name=map_name, folder=map_folder, imsize=desired_map_size)
            # Call the point cloud and bev generations
            map_ptc = robot_map_manager.generateMapPtc()
            world_T_map = robot_map_manager.computeWorldTMap()
            map_bev, map_coords_json = robot_map_manager.generateMapBev(
                ptc_map_frame=map_ptc, world_T_map=world_T_map)
            # Save the files
            robot_map_manager.saveMap(
                map_bev=map_bev, map_ptc=map_ptc, map_coords=map_coords_json, world_T_map=world_T_map)

        # Set the reference transformation if it is the first map
        if world_T_map_ref is None:
            world_T_map_ref = world_T_map

        # Transform the point cloud to the world frame
        map_ptc_world_frame = deepcopy(map_ptc).transform(world_T_map)

        # Add the point cloud to the global map
        global_map_point_cloud += map_ptc_world_frame

    # Create a map manager object for the global map and save the entities
    print("Creating global map ...")
    global_map_manager = MapManager(
        map_name=global_map_name, folder=global_map_folder, imsize=[1080, 1920])
    global_map_bev, global_map_coords_json = global_map_manager.generateMapBev(
        ptc_map_frame=global_map_point_cloud, world_T_map=world_T_map_ref)

    print("Saving global map ...")
    global_map_manager.saveMap(map_bev=global_map_bev, map_ptc=global_map_point_cloud,
                               map_coords=global_map_coords_json, world_T_map=world_T_map_ref)
    print("Global map created and saved!")


if __name__ == "__main__":
    maps_folder = os.path.join(os.getenv("HOME"), "maps")
    main(maps_folder)
