from controller import Robot, Lidar, Compass, GPS, Keyboard, Supervisor
import numpy as np

from controllers.transformations import create_tf_matrix, get_translation
from controllers.utils import cmd_vel, plot_line, teletransporte, record_lidar_scan

from deterministic_occupancy_grid import *
from ransac_functions import RansacPrimitiveClassifier
from constants import *
from kmeans import Kmeans


def main() -> None:
    robot: Supervisor = Supervisor()
    timestep: int = ROBOT_TIMESTEP  # in ms

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
        elif key == ord('T'):
            scan_count, current_count = teletransporte(robot, scan_count, gps, compass,
                                                       lidar, map, current_count, timestep)
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
                shape_result = [lines, ransac_count, x, y]
                print(f"Num of ransac runs: {ransac_count}")
                plot_line(x, y)
                shape_measures = ransac.get_shape_measures(ransac_count, x, y)
                shape_result.append(shape_measures)
                results.append(shape_result)

            return

        cmd_vel(robot, lin_vel, ang_vel)
        if record_lidar_scan(current_count, gps, compass, lidar, map):
            current_count = 0
            scan_count += 1
            print("scan count: ", scan_count)
        current_count += 1





if __name__ == '__main__':
    main()
