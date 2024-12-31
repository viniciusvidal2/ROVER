from typing import Tuple, Dict
import numpy as np
import open3d as o3d
import os
import cv2
import msgpack


class MapManager:
    def __init__(self, rover_code: int = 0, folder: str = "", imsize: list = [1080, 1920]) -> None:
        self.rover_code = rover_code
        self.map_folder = folder
        self.image_size = imsize

    def generateMapPtc(self) -> o3d.geometry.PointCloud:
        map_point_cloud = o3d.geometry.PointCloud()
        for filename in os.listdir(self.map_folder):
            if filename.endswith(".pcd") and filename.startswith("cloud"):
                file_path = os.path.join(self.map_folder, filename)
                map_point_cloud += o3d.io.read_point_cloud(file_path)
        return map_point_cloud

    def generateMapBev(self, ptc_map_frame: o3d.geometry.PointCloud) -> Tuple[np.ndarray, Dict]:
        if (len(np.asarray(ptc_map_frame.points)) == 0):
            return np.ndarray()

        # Get the boundaries and extent from the map point cloud in map frame
        aabb = ptc_map_frame.get_axis_aligned_bounding_box()
        min_xy_offset = aabb.get_min_bound()[:2]
        # y,x -> lines, collumns in the image
        yx_extent = (aabb.get_max_bound()[:2] - min_xy_offset)[::-1]
        # Find the resolution that composes the proposed image resolution without changing map aspect ratio
        resolution_per_axis = self.image_size / yx_extent  # [pixel/meter]
        resolution = np.min(resolution_per_axis)
        # Fixes the image size to be exact what it takes to generate the image
        self.image_size = np.ceil(yx_extent * resolution).astype(int)

        # Get the transformation from world to map
        map_T_world = np.array([[-0.5467,      -0.8373,       0.0000, 6721299.1559],
                                [0.8373,      -0.5467,       0.0000, 3590546.8287],
                                [0.0000,       0.0000,       1.0000,    -926.5576],
                                [0.0000,       0.0000,       0.0000,       1.0000]])
        world_T_map = np.linalg.inv(map_T_world)

        # Create the image to project the points and run the projection
        # Get the global coordinates of each projected pixel as well
        map_bev = np.zeros(shape=tuple(self.image_size), dtype=np.uint8)
        map_z = np.zeros(shape=tuple(self.image_size), dtype=float)
        keys = [f"{v}_{u}" for u in range(self.image_size[1]) for v in range(self.image_size[0])]
        default_coords = {"lat": 0, "lon": 0, "alt": 0}
        map_coords = dict.fromkeys(keys, default_coords)

        for p_map, p_color in zip(np.asarray(ptc_map_frame.points), np.asarray(ptc_map_frame.colors)):
            # Operation to get the image coordinates based on the xy values in the map
            [u, v] = np.floor((p_map[:2] - min_xy_offset)
                              * resolution).astype(int) - 1

            # Only project if the point ir higher than the previous one, or no projection is in the pixel yet
            if map_z[v, u] == 0 or p_map[2] > map_z[u, v]:
                map_bev[v, u] = np.uint8(p_color[0] * 255)

                # Transform to UTM and then latlon
                p_map_h = np.append(p_map, 1)
                p_world_utm = world_T_map @ p_map_h
                utm_e, utm_n, alt = p_world_utm[0], p_world_utm[1], p_world_utm[2]
                lat, lon = utm.to_latlon(
                    easting=utm_e, northing=utm_n, zone_number=23, zone_letter='l')
                # Save to map dictionary
                key = f"{v}_{u}"
                coords = {"lat": lat, "lon": lon, "alt": alt}
                map_coords[key] = coords

        # Apply morphological operations and light enhancement for the BEV map
        kernel = np.ones((3, 3), np.uint8)
        map_bev = cv2.dilate(map_bev, kernel, iterations=1)
        map_bev = cv2.equalizeHist(map_bev)

        # Get the output JSON together
        return map_bev, map_coords

    def saveMap(self, map_bev: np.ndarray, map_ptc: o3d.geometry.PointCloud, map_coords: dict) -> None:
        # Rover name from the code
        rover_name = "rover_" + str(self.rover_code)

        # Save the BEV image
        if len(map_bev > 0):
            bev_path = os.path.join(self.map_folder, rover_name + ".png")
            cv2.imwrite(bev_path, map_bev)

        # Save the map point cloud
        if (len(np.asarray(map_ptc.points)) > 0):
            ptc_path = os.path.join(self.map_folder, rover_name + ".pcd")
            o3d.io.write_point_cloud(ptc_path, map_ptc, write_ascii=False)

        # Save the JSON file with coordinates
        if len(map_coords) > 0:
            coords_path = os.path.join(
                self.map_folder, rover_name + ".msgpack")
            with open(coords_path, "wb") as msgpack_file:
                msgpack.dump(map_coords, msgpack_file)

    def smoothFillFloatImage(self, image: np.ndarray, kernel_size: int, iterations: int) -> np.ndarray:
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        return cv2.dilate(image, kernel, iterations=iterations)

    def enhanceImageQuality(self, image: np.ndarray, kernel_size: int, iterations: int) -> np.ndarray:
        smoothed_image = self.smoothFillFloatImage(
            image=image, kernel_size=kernel_size, iterations=iterations)
        return cv2.equalizeHist(smoothed_image)
