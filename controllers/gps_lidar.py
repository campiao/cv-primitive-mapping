import math

from matplotlib import pyplot as plt
import pyransac3d as pyrsc
from controller import Robot, Lidar, LidarPoint, Compass, GPS, Keyboard
import numpy as np
from occupancy_grid import OccupancyGrid
from controllers.transformations import create_tf_matrix, get_translation
from controllers.utils import cmd_vel, bresenham

LIDAR_SCAN_UPDATES = 1
LIDAR_HORIZONTAL_RESOLUTION = 200
LIDAR_FOV = 6.28

GRID_RESOLUTION = 0.001


class DeterministicOccupancyGrid(OccupancyGrid):
    def __init__(self, origin: (float, float), dimensions: (int, int), resolution: float):
        super().__init__(origin, dimensions, resolution)

        # Initialize the grid
        self.x = []
        self.y = []

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

        return self.x, self.y

    def get_map(self):
        return self.x, self.y

    def to_string(self) -> str:
        return f"x: {self.x}, y: {self.y}"


def main() -> None:
    robot: Robot = Robot()
    timestep: int = 100  # in ms

    kb: Keyboard = Keyboard()
    kb.disable()
    kb.enable(timestep)

    keyboard_linear_vel: float = 0.3
    keyboard_angular_vel: float = 1.5

    map: DeterministicOccupancyGrid = DeterministicOccupancyGrid([0.0, 0.0], [200, 200], GRID_RESOLUTION)

    lidar: Lidar = robot.getDevice('lidar')
    lidar.enable(timestep)
    lidar.enablePointCloud()

    compass: Compass = robot.getDevice('compass')
    compass.enable(timestep)

    gps: GPS = robot.getDevice('gps')
    gps.enable(timestep)

    scan_count: int = 0
    current_count: int = 0
    while robot.step(timestep) != -1:
        lin_vel: float = 0
        ang_vel: float = 0
        key: int = kb.getKey()
        if key == ord('W'):
            lin_vel += keyboard_linear_vel
        elif key == ord('S'):
            lin_vel -= keyboard_linear_vel
        elif key == ord('A'):
            ang_vel += keyboard_angular_vel
        elif key == ord('D'):
            ang_vel -= keyboard_angular_vel
        elif key == ord('P'):
            x, y = map.get_map()
            # Crie um novo gráfico 2D
            plt.figure()

            # Plote os pontos no gráfico 2D
            plt.scatter(x, y, s=1)

            # Configure os rótulos dos eixos
            plt.xlabel('X')
            plt.ylabel('Y')
            # Exiba o gráfico
            plt.show()
        elif key == ord('L'):
            nova_lista = []
            for i in range(len(x)):
                par = [x[i], y[i], 1]  # Cria uma lista com os números emparelhados e adiciona 1
                nova_lista.append(par)
            lidar_data_processed = np.array(nova_lista)
            # Load your point cloud as a numpy array (N, 3)

            sph = pyrsc.Circle()
            center, axis, radius, inliers = sph.fit(lidar_data_processed, thresh=0.05, maxIteration=1000)
            print(f"center: {center}, radius: {radius}")
            print(f"adjusted center: {map.grid_to_real_coords(center)}, radius: {radius * GRID_RESOLUTION}")
            return

        cmd_vel(robot, lin_vel, ang_vel)
        if record_lidar_scan(current_count, gps, compass, lidar, map):
            current_count = 0
            scan_count += 1
            print("scan count: ", scan_count)
        current_count += 1


def record_lidar_scan(current_count, gps, compass, lidar, map):
    if current_count < LIDAR_SCAN_UPDATES:
        return False

    # Read the robot's pose
    gps_readings: [float] = gps.getValues()
    robot_position: (float, float) = (gps_readings[0], gps_readings[1])
    compass_readings: [float] = compass.getValues()
    robot_orientation: float = math.atan2(compass_readings[0], compass_readings[1])
    robot_tf: np.ndarray = create_tf_matrix((robot_position[0], robot_position[1], 0.0), robot_orientation)

    # Read the LiDAR and update the map
    x, y = map.update_map(robot_tf, lidar.getPointCloud())

    return True


if __name__ == '__main__':
    main()
