import math
import numpy as np

from occupancy_grid import OccupancyGrid
from controller import LidarPoint
from controllers.transformations import get_translation


class DeterministicOccupancyGrid(OccupancyGrid):
    def __init__(self, origin: (float, float), dimensions: (int, int), resolution: float):
        super().__init__(origin, dimensions, resolution)

        # Initialize the grid
        self.x = []
        self.y = []
        self.grid = set()

    def update_map(self, robot_tf: np.ndarray, lidar_points: [LidarPoint]):
        # Get the grid coord for the robot pose
        robot_coord: (int, int) = self.real_to_grid_coords(get_translation(robot_tf)[0:2])

        # Get the grid coords for the lidar points
        grid_lidar_coords: [(int, int)] = []
        for point in lidar_points:
            if math.isfinite(point.x) and math.isfinite(point.y):
                coord: (int, int) = self.real_to_grid_coords(np.dot(robot_tf, [point.x, point.y, 0.0, 1.0])[0:2])
                grid_lidar_coords.append(coord)
        # Set as occupied the cells for the lidar points
        for coord in grid_lidar_coords:
            self.x.append(coord[0])
            self.y.append(coord[1])
            self.grid.add(coord)

        return self.x, self.y

    def get_map(self):
        return self.x, self.y

    def get_grid(self):
        return list(self.grid)

    def get_x_y_coord(self):
        points = self.get_grid()
        x = [point[0] for point in points]
        y = [point[1] for point in points]
        return x, y

    def to_string(self) -> str:
        return f"x: {self.x}, y: {self.y}"
