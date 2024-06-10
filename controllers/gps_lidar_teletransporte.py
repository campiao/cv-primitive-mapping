import math

from matplotlib import pyplot as plt
import pyransac3d as pyrsc
from controller import Robot, Lidar, LidarPoint, Compass, GPS, Keyboard, Supervisor
import numpy as np
from occupancy_grid import OccupancyGrid
from controllers.transformations import create_tf_matrix, get_translation
from controllers.utils import cmd_vel, warp_robot, bresenham
from ransac_functions import ransac_fit_line

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

    def to_string(self) -> str:
        return f"x: {self.x}, y: {self.y}"


def main() -> None:
    robot: Supervisor = Supervisor()
    timestep: int = 100  # in ms

    kb: Keyboard = Keyboard()
    kb.disable()
    kb.enable(timestep)

    keyboard_linear_vel: float = 5.0
    keyboard_angular_vel: float = 3.0

    map: DeterministicOccupancyGrid = DeterministicOccupancyGrid([0.0, 0.0], [200, 200], GRID_RESOLUTION)

    lidar: Lidar = robot.getDevice('lidar')
    lidar.enable(timestep)
    lidar.enablePointCloud()

    compass: Compass = robot.getDevice('compass')
    compass.enable(timestep)

    gps: GPS = robot.getDevice('gps')
    gps.enable(timestep)

    tele_distance = 0.1
    waypoints_horizontal = []
    waypoints_vertical = []
    coordXh = 0.1
    coordYh = 0.1
    while coordYh < 1.8:
        waypoints_horizontal.append((coordXh, coordYh))
        waypoints_vertical.append((coordYh, coordXh))
        coordXh += tele_distance
        if coordXh > 1.8:
            coordXh = 0.1
            coordYh += tele_distance

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
            x, y = plot_points(map)

        elif key == ord('L'):
            nova_lista = []
            for i in range(len(x)):
                par = [x[i], y[i], 1]  # Cria uma lista com os números emparelhados e adiciona 1
                nova_lista.append(par)
            lidar_data_processed = np.array(nova_lista)
            # Load your point cloud as a numpy array (N, 3)
            readings = np.array([[x[i], y[i]] for i in range(len(x))])

            inliers, outliers, line_data = ransac_fit_line(readings)
            x = [point[0] for point in inliers]
            y = [point[1] for point in inliers]
            lines = [line_data]
            ransac_count = 1
            while len(outliers) > 500:
                inliers, outliers, line_data = ransac_fit_line(outliers)
                for line in lines:
                    if line_data.params[0][0] - line.params[0][0] == 0:
                        declive = 0
                    else:
                        declive = (line_data.params[0][1] - line.params[0][1]) / (
                                line_data.params[0][0] - line.params[0][0])
                    if line.params[1][0] == 0:
                        stored_slope = 0
                    else:
                        stored_slope = line.params[1][1] / line.params[1][0]
                    if stored_slope == 0:
                        print("skipped, count", ransac_count)
                        continue
                    if declive - stored_slope < 0.01:
                        print("skipped, count", ransac_count)
                        continue
                lines.append(line_data)
                for point in inliers:
                    x.append(point[0])
                    y.append(point[1])
                ransac_count += 1
                print("not skipped, count", ransac_count)

            print(f"Num of ransac runs: {ransac_count}")
            dir_vectors = [[math.floor(data.params[1][0]), math.floor(data.params[1][1])] for data in lines]
            dir_vectors = np.array(dir_vectors)
            print(dir_vectors)
            unique = np.unique(dir_vectors, axis=0)
            print(f"Unique direction vectors: {np.count_nonzero(unique)}")
            print(unique)

            print()

            # Crie um novo gráfico 2D
            plt.figure()

            # Plote os pontos no gráfico 2D
            plt.scatter(x, y, s=1)

            # Configure os rótulos dos eixos
            plt.xlabel('X')
            plt.ylabel('Y')
            # Exiba o gráfico
            plt.show()

            # sph = pyrsc.Circle()
            # center, axis, radius, inliers = sph.fit(lidar_data_processed, thresh=0.05, maxIteration=1000)
            # print(f"center: {center}, radius: {radius}")
            # print(f"adjusted center: {map.grid_to_real_coords(center)}, radius: {radius * GRID_RESOLUTION}")
            return

        elif key == ord('T'):
            for waypoint in waypoints_horizontal:
                warp_robot(robot, "EPUCK", waypoint)
                robot.step(10)

                if record_lidar_scan(current_count, gps, compass, lidar, map):
                    current_count = 0
                    scan_count += 1
                    print("scan count: ", scan_count)
                current_count += 1

            # Gira o robô 180 graus
            rotate_robot_180(robot, keyboard_angular_vel)

            # Percorre os waypoints na ordem inversa
            for waypoint in reversed(waypoints_horizontal):

                warp_robot(robot, "EPUCK", waypoint)
                robot.step(10)

                if record_lidar_scan(current_count, gps, compass, lidar, map):
                    current_count = 0
                    scan_count += 1
                    print("scan count: ", scan_count)
                current_count += 1

            # Gira o robô 90 graus
            rotate_robot_90(robot, keyboard_angular_vel)
            for waypoint in waypoints_vertical:
                warp_robot(robot, "EPUCK", waypoint)
                robot.step(10)

                if record_lidar_scan(current_count, gps, compass, lidar, map):
                    current_count = 0
                    scan_count += 1
                    print("scan count: ", scan_count)
                current_count += 1




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


def rotate_robot_180(robot, angular_velocity):
    # Define o tempo necessário para girar 180 graus
    rotation_time = math.pi / angular_velocity
    # Gira o robô
    cmd_vel(robot, 0, angular_velocity)
    # Aguarda o tempo necessário para a rotação
    robot.step(100)
    # Para o robô
    cmd_vel(robot, 0, 0)

def rotate_robot_90(robot, angular_velocity):
    # Define o tempo necessário para girar 180 graus
    rotation_time = math.pi / angular_velocity
    # Gira o robô
    cmd_vel(robot, 0, angular_velocity)
    # Aguarda o tempo necessário para a rotação
    robot.step(100)
    # Para o robô
    cmd_vel(robot, 0, 0)

def plot_points(map):
    points = map.get_grid()
    x = [point[0] for point in points]
    y = [point[1] for point in points]
    print(points)

    # Crie um novo gráfico 2D
    plt.figure()

    # Plote os pontos no gráfico 2D
    plt.scatter(x, y, s=1)

    # Configure os rótulos dos eixos
    plt.xlabel('X')
    plt.ylabel('Y')
    # Exiba o gráfico
    plt.show()
    return x, y


if __name__ == '__main__':
    main()
