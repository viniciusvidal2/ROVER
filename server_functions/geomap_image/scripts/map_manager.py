from typing import Tuple, Dict
import numpy as np
import open3d as o3d
import os
import cv2
import msgpack


class MapManager:
    def __init__(self, rover_code: int = 0, folder: str = "", imsize: list = [1080, 1920]) -> None:
        """Constructor

        Args:
            rover_code (int, optional): The rover id. Defaults to 0.
            folder (str, optional): The map folder from the rover. Defaults to "".
            imsize (list, optional): The output image map size [rows, cols]. Defaults to [1080, 1920].
        """
        self.rover_code = rover_code
        self.map_folder = folder
        self.image_size = imsize

    def generateMapPtc(self) -> o3d.geometry.PointCloud:
        """Reads all the point cloud tiles and returns the entire map cloud

        Returns:
            o3d.geometry.PointCloud: the map cloud
        """
        map_point_cloud = o3d.geometry.PointCloud()
        for filename in os.listdir(self.map_folder):
            if filename.endswith(".pcd") and filename.startswith("cloud"):
                file_path = os.path.join(self.map_folder, filename)
                map_point_cloud += o3d.io.read_point_cloud(file_path)
        return map_point_cloud

    def generateMapBev(self, ptc_map_frame: o3d.geometry.PointCloud) -> Tuple[np.ndarray, Dict]:
        """Generates the map birds eye view image, along with georeferenced data

        Args:
            ptc_map_frame (o3d.geometry.PointCloud): The map point cloud

        Returns:
            Tuple[np.ndarray, Dict]: bev image, dict with georef data
        """
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

        # Output variables
        map_bev = np.zeros(shape=tuple(self.image_size), dtype=np.uint8)
        map_z = np.zeros(shape=tuple(self.image_size), dtype=float)
        keys = [f"{v}_{u}" for u in range(
            self.image_size[1]) for v in range(self.image_size[0])]
        default_coords = {"utm_e": 0, "utm_n": 0, "alt": 0}
        map_coords = dict.fromkeys(keys, default_coords)

        # Transform all the points to world frame
        ptc_map_frame_points = np.asarray(ptc_map_frame.points)
        points_map_frame_h = np.hstack(
            [ptc_map_frame_points, np.ones((ptc_map_frame_points.shape[0], 1))])
        points_world_frame_h = (world_T_map @ points_map_frame_h.T).T

        ptc_point_colors = np.asarray(ptc_map_frame.colors)
        for p_map, p_color, p_world in zip(ptc_map_frame_points, ptc_point_colors, points_world_frame_h):
            # Operation to get the image coordinates based on the xy values in the map
            [u, v] = np.floor((p_map[:2] - min_xy_offset)
                              * resolution).astype(int) - 1

            # Only project if the point ir higher than the previous one, or no projection is in the pixel yet
            if map_z[v, u] == 0 or p_map[2] > map_z[v, u]:
                map_bev[v, u] = np.uint8(p_color[0] * 255)
                map_z[v, u] = p_map[2]

                # Save to map dictionary
                utm_e, utm_n, alt = p_world[0], p_world[1], p_world[2]
                key = f"{v}_{u}"
                coords = {"utm_e": utm_e, "utm_n": utm_n, "alt": alt}
                map_coords[key] = coords

        # The coordinates that still contain zeros in Z image should be filled as much as possible
        map_z = self.smoothFillImage(
            image=map_z, kernel_size=7, iterations=5)
        # If we ever find a missing height value, use the last valid one as an approximation
        last_valid_z = 0
        # Go looking for missing utm data, and fill in with an approximation based on the pixel value
        for v in range(map_bev.shape[0]):
            for u in range(map_bev.shape[1]):
                if map_z[v, u] != 0:
                    last_valid_z = map_z[v, u]

                key = f"{v}_{u}"
                if map_coords[key]["utm_e"] == 0:
                    # Find the map coordinates using the pixel value and resolution,
                    # then use Z and transform to get the point in world frame
                    xy_map_frame = min_xy_offset + \
                        np.array([u, v]) * resolution
                    z_map_frame = map_z[v, u] if map_z[v,
                                                       u] != 0 else last_valid_z
                    xyz_map_frame_h = np.hstack(
                        [xy_map_frame, np.array([z_map_frame, 1])])
                    p_world = world_T_map @ xyz_map_frame_h
                    # Save to map dictionary
                    utm_e, utm_n, alt = p_world[0], p_world[1], p_world[2]
                    map_coords[key] = {"utm_e": utm_e,
                                       "utm_n": utm_n, "alt": alt}

        # Apply morphological operations and light enhancement for the BEV map
        map_bev = self.enhanceImageQuality(
            image=map_bev, kernel_size=3, iterations=1)

        # Get the output JSON together
        return map_bev, map_coords

    def saveMap(self, map_bev: np.ndarray, map_ptc: o3d.geometry.PointCloud, map_coords: dict) -> None:
        """Saves the map data

        Args:
            map_bev (np.ndarray): map birds eye view image, saved as png
            map_ptc (o3d.geometry.PointCloud): map point cloud, saved as pcd
            map_coords (dict): map georef data, saved as msgpack
        """
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

    def smoothFillImage(self, image: np.ndarray, kernel_size: int, iterations: int) -> np.ndarray:
        """Smooth float image to fill in some zeros

        Args:
            image (np.ndarray): input float data map
            kernel_size (int): kernel size to run dilation operations
            iterations (int): number of dilations

        Returns:
            np.ndarray: the filled float image
        """
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        return cv2.dilate(image, kernel, iterations=iterations)

    def enhanceImageQuality(self, image: np.ndarray, kernel_size: int, iterations: int) -> np.ndarray:
        """Improve image quality with operations

        Args:
            image (np.ndarray): input image
            kernel_size (int): kernel size
            iterations (int): number of iterations

        Returns:
            np.ndarray: improved image
        """
        smoothed_image = self.smoothFillImage(
            image=image, kernel_size=kernel_size, iterations=iterations)
        return cv2.equalizeHist(smoothed_image)
