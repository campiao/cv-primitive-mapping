from controller import Lidar, Compass, GPS, Keyboard, Supervisor
from controllers.solver import solve_shapes_problem
from controllers.utils import cmd_vel, plot_line, teletransporte, record_lidar_scan, save_results
import time
from deterministic_occupancy_grid import *
from constants import *


def main(filename, num_shapes) -> None:
    robot: Supervisor = Supervisor()
    timestep: int = ROBOT_TIMESTEP  # in ms

    kb: Keyboard = Keyboard()
    kb.disable()
    kb.enable(timestep)

    start_time = None

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
        if key != -1 and start_time is None:
            start_time = time.time()
            print("Starting time...")
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
            results = solve_shapes_problem(map, filename, num_shapes)
            elapsed_time = 0
            if start_time is not None:
                elapsed_time = time.time() - start_time
                elapsed_time = round(elapsed_time, 2)
                print(f"Time: {elapsed_time:.2f} seconds")
            results.append(elapsed_time)
            save_results(filename, results, num_shapes)
            return

        cmd_vel(robot, lin_vel, ang_vel)
        if record_lidar_scan(current_count, gps, compass, lidar, map):
            current_count = 0
            scan_count += 1
            print("scan count: ", scan_count)
        current_count += 1


if __name__ == '__main__':
    map_name=input("Map name, without extension:")
    shapes=input("Number of shapes:")
    main(map_name,int(shapes))

