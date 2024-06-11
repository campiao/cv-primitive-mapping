from controller import Robot, Lidar, Compass, GPS, Keyboard
import numpy as np

from controllers.transformations import create_tf_matrix, get_translation
from controllers.utils import cmd_vel, plot_line

from deterministic_occupancy_grid import *
from ransac_functions import RansacPrimitiveClassifier
from constants import *
from kmeans import Kmeans


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
            x, y = map.get_x_y_coord()
            plot_line(x, y)

        elif key == ord('L'):
            x, y = map.get_x_y_coord()
            # Load your point cloud as a numpy array (N, 3)
            readings = np.array([[x[i], y[i]] for i in range(len(x))])
            x = readings[:, 0]
            y = readings[:, 1]

            kmeans = Kmeans()
            shapes = kmeans.Kmeans(x, y, 2)
            print(len(shapes))

            ransac = RansacPrimitiveClassifier()
            results = []
            for shape in shapes:
                lines, ransac_count, x, y = ransac.solve_shape(shape, PERCENT_OF_TOTAL)
                results.append([lines, ransac_count, x, y])

                print(f"Num of ransac runs: {ransac_count}")
                plot_line(x, y)
                ransac.get_shape_measures(ransac_count, x, y)

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
