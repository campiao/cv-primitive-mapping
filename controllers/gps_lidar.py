from matplotlib import pyplot as plt
import pyransac3d as pyrsc
from controller import Robot, Lidar, Compass, GPS, Keyboard
import numpy as np

from controllers.transformations import create_tf_matrix, get_translation
from controllers.utils import cmd_vel, bresenham

from deterministic_occupancy_grid import *
from ransac_functions import RansacPrimitiveClassifier

LIDAR_SCAN_UPDATES = 1
LIDAR_HORIZONTAL_RESOLUTION = 200
LIDAR_FOV = 6.28

GRID_RESOLUTION = 0.001
PERCENT_OF_TOTAL = 0.4


def main() -> None:
    robot: Robot = Robot()
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
            points = map.get_grid()
            x = [point[0] for point in points]
            y = [point[1] for point in points]
            print(points)
            plot_line(x, y)

        elif key == ord('L'):
            nova_lista = []
            for i in range(len(x)):
                par = [x[i], y[i], 1]  # Cria uma lista com os números emparelhados e adiciona 1
                nova_lista.append(par)
            lidar_data_processed = np.array(nova_lista)
            # Load your point cloud as a numpy array (N, 3)
            readings = np.array([[x[i], y[i]] for i in range(len(x))])

            ransac = RansacPrimitiveClassifier()
            lines, ransac_count, x, y = ransac.solve_shape(readings, PERCENT_OF_TOTAL)

            print(f"Num of ransac runs: {ransac_count}")
            dir_vectors = [[math.floor(data.params[1][0]), math.floor(data.params[1][1])] for data in lines]
            dir_vectors = np.array(dir_vectors)
            print(dir_vectors)
            unique = np.unique(dir_vectors, axis=0)
            print(f"Unique direction vectors: {np.count_nonzero(unique)}")
            print(unique)

            print()
            plot_line(x, y)

            # sph = pyrsc.Circle()
            # center, axis, radius, inliers = sph.fit(lidar_data_processed, thresh=0.05, maxIteration=1000)
            # print(f"center: {center}, radius: {radius}")
            # print(f"adjusted center: {map.grid_to_real_coords(center)}, radius: {radius * GRID_RESOLUTION}")
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


def plot_line(x, y):
    # Crie um novo gráfico 2D
    plt.figure()

    # Plote os pontos no gráfico 2D
    plt.scatter(x, y, s=1)

    # Configure os rótulos dos eixos
    plt.xlabel('X')
    plt.ylabel('Y')
    # Exiba o gráfico
    plt.show()


if __name__ == '__main__':
    main()
