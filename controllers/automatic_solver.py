import time

from constants import *
from deterministic_occupancy_grid import DeterministicOccupancyGrid
from utils import teletransporte, initialize_webots_features
from gps_lidar import solve_shapes_problem


def automatic_solver(map_name):
    robot, lidar, compass, gps = initialize_webots_features()
    grid: DeterministicOccupancyGrid = DeterministicOccupancyGrid([0.0, 0.0], [200, 200], GRID_RESOLUTION)

    scan_count: int = 0
    current_count: int = 0
    start_time = time.time()
    scan_count, current_count = teletransporte(robot, scan_count, gps, compass,lidar, grid, current_count,
                                               ROBOT_TIMESTEP)
    solve_shapes_problem(grid, map_name)

    if start_time is not None:
        elapsed_time = time.time() - start_time
        print(f"Time: {elapsed_time:.2f} seconds")

if __name__ == '__main__':
    automatic_solver("try")