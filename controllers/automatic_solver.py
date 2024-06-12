import time

from constants import *
from controllers.solver import solve_shapes_problem
from deterministic_occupancy_grid import DeterministicOccupancyGrid
from utils import teletransporte, initialize_webots_features, save_results


def automatic_solver(map_name, num_shapes):
    robot, lidar, compass, gps = initialize_webots_features()
    grid: DeterministicOccupancyGrid = DeterministicOccupancyGrid([0.0, 0.0], [200, 200], GRID_RESOLUTION)

    scan_count: int = 0
    current_count: int = 0
    start_time = time.time()
    scan_count, current_count = teletransporte(robot, scan_count, gps, compass, lidar, grid, current_count,
                                               ROBOT_TIMESTEP)
    results = solve_shapes_problem(grid, map_name, num_shapes)

    elapsed_time = time.time() - start_time
    elapsed_time = round(elapsed_time, 2)
    print(f"Time: {elapsed_time:.2f} seconds")
    results.append(elapsed_time)
    save_results(map_name, results, num_shapes)


if __name__ == '__main__':
    automatic_solver("triangulo", 1)
