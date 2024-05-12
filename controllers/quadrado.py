import math
import csv
import math

import numpy as np
from matplotlib import pyplot as plt
from numpy import array
from skimage.measure import LineModelND, ransac

from controller import Robot, LidarPoint, Lidar, Compass, GPS, Keyboard
from controllers.localization_utils import draw_real_vs_estimated_localization
from controllers.transformations import create_tf_matrix, get_translation, get_rotation
from utils import cmd_vel

import pyransac3d as pyrsc
import numpy as np
from matplotlib import pyplot as plt
from numpy import array
from skimage.measure import LineModelND, ransac

from controllers.transformations import create_tf_matrix, get_translation, get_rotation

def read_coordinates_from_csv(file_path):
    coordinates = []
    with open(file_path, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            if len(row) >= 2:
                x, y = map(float, row[:2])  # Assuming first two columns are x, y coordinates
                coordinates.append([x, y])
    return np.array(coordinates)

def find_possible_poses_from_csv(coordinates: np.ndarray, num_lines: int) -> ([(float, float)], [float]):
    # Structure the coordinates for input to RANSAC
    data = array(coordinates)

    max_iterations = num_lines  # Define the number of iterations

    models = []
    inliers_bools = []
    outliers = []
    original_data = data.copy()  # Preserve the original data

    for i in range(max_iterations):
        # Find the line
        model, inliers_bool = ransac(data, LineModelND, min_samples=2,
                                     residual_threshold=0.007, max_trials=10000)
        models.append(model)
        inliers_bools.append(inliers_bool)

        # Retrieve the outliers
        outliers_data = array([point for (point, inlier_bool) in zip(data, inliers_bool) if not inlier_bool])
        outliers.append(outliers_data)

        # Update data for the next iteration
        data = outliers_data

    # Compute the inliers for each iteration
    inliers = [array([point for (point, inlier_bool) in zip(original_data, inliers_bool) if inlier_bool]) for
               original_data, inliers_bool in zip([original_data] + outliers, inliers_bools)]

    # Run the final function with the collected data
    draw_walls(original_data, models, inliers, outliers[-1])

def draw_walls(data: array, models: array(LineModelND), inliers: array([array]), outliers: array) -> None:
    # Unpack the points into separate lists of x and y coordinates
    print(data)
    data_x, data_y = zip(*data)

    inliers_x = []
    inliers_y = []

    for inlier in inliers:
        inlier_x, inlier_y = zip(*inlier)
        inliers_x.append(inlier_x)
        inliers_y.append(inlier_y)

    outliers_x = []
    outliers_y = []
    if len(outliers) > 0:
        outliers_x, outliers_y = zip(*outliers)

    # Plot the data points with different colors
    for i, (model, inlier_x, inlier_y) in enumerate(zip(models, inliers_x, inliers_y)):
        plt.scatter(inlier_x, inlier_y, color='blue', label=f'Inliers{i + 1}', marker='x')

        # Add the computed line to the plot
        line_x_values, line_y_values = get_model_line_points(inlier_x, inlier_y, model.params[0], model.params[1])
        plt.plot(line_x_values, line_y_values, color='blue', label=f'Line{i + 1}')

    # Add outliers to the plot
    if len(outliers) > 0:
        plt.scatter(outliers_x, outliers_y, color='red', label='Outliers', marker='x')

    # Add labels and legend
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('LiDAR points and lines relative to the robot')
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.gca().set_aspect('equal')
    plt.xlim(min(*data_x, 0) - 0.1, max(*data_x, 0) + 0.1)
    plt.ylim(min(*data_y, 0) - 0.1, max(*data_y, 0) + 0.1)

    # Show the plot
    plt.grid(True)
    plt.show()

def get_model_line_points(points_x_coords: [float], points_y_coords: [float], origin: (float, float),
                          direction: (float, float)) -> (np.ndarray, np.ndarray):
    if direction[0] == 0:  # vertical line
        line_min_y: float = min(points_y_coords)
        line_max_y: float = max(points_y_coords)
        line_y_values: np.ndarray[np.dtype:float] = np.linspace(line_min_y - 0.1, line_max_y + 0.1, 100)

        line_x_values: np.ndarray[np.dtype:float] = np.full(shape=len(line_y_values), fill_value=points_x_coords[0],
                                                            dtype=np.float64)
    else:
        line_min_x: float = min(points_x_coords)
        line_max_x: float = max(points_x_coords)
        line_x_values: np.ndarray[np.dtype:float] = np.linspace(line_min_x - 0.1, line_max_x + 0.1, 100)

        slope: float = direction[1] / direction[0]
        intercept: float = origin[1] - slope * origin[0]
        line_y_values: np.ndarray[np.dtype:float] = slope * line_x_values + intercept
    return line_x_values, line_y_values


import math

import numpy as np
from matplotlib import pyplot as plt
from numpy import array
from skimage.measure import LineModelND, ransac

from controller import Robot, LidarPoint, Lidar, Compass, GPS, Keyboard
from controllers.localization_utils import draw_real_vs_estimated_localization
from controllers.transformations import create_tf_matrix, get_translation, get_rotation
from utils import cmd_vel

import pyransac3d as pyrsc


def main() -> None:
    lidar_readings = []
    scan_count = 0
    robot: Robot = Robot()
    timestep: int = 100
    kb: Keyboard = Keyboard()
    kb.enable(timestep)

    keyboard_linear_vel: float = 0.3
    keyboard_angular_vel: float = 1.5

    min_x: float = -0.5
    min_y: float = -0.5
    max_x: float = 0.5
    max_y: float = 0.5

    timestep: int = int(robot.getBasicTimeStep())  # in ms

    lidar: Lidar = robot.getDevice('lidar')
    lidar.enable(int(robot.getBasicTimeStep()))
    lidar.enablePointCloud()

    compass: Compass = robot.getDevice('compass')
    compass.enable(timestep)

    gps: GPS = robot.getDevice('gps')
    gps.enable(timestep)
    robot.step()

    # Read the ground-truth (correct robot pose)
    gps_readings: [float] = gps.getValues()
    actual_position: (float, float) = (gps_readings[0], gps_readings[1])
    compass_readings: [float] = compass.getValues()
    actual_orientation: float = math.atan2(compass_readings[0], compass_readings[1])

    robot_tf: np.ndarray = create_tf_matrix((actual_position[0], actual_position[1], 0.0), actual_orientation)

    # Draw a point cloud for a square map
    num_divisions: int = 100
    fixed_points: [(float, float, float)] = []
    for i in range(num_divisions):
        x: float = min_x + (max_x - min_x) * (i / float(num_divisions - 1))
        fixed_points.append([x, min_y, 0.0])
        fixed_points.append([x, max_y, 0.0])

        y: float = min_y + (max_y - min_y) * (i / float(num_divisions - 1))
        fixed_points.append([min_x, y, 0.0])
        fixed_points.append([max_x, y, 0.0])
    fixed_cloud: np.ndarray = np.asarray(fixed_points)

    # estimated_translations, estimated_rotations = find_possible_poses(robot_tf, lidar.getPointCloud(), min_x, max_x, min_y, max_y)
    # draw_real_vs_estimated_localization(fixed_cloud,
    #                                   actual_position, actual_orientation,
    #                                    estimated_translations, estimated_rotations)

    while robot.step() != -1:
        lidar_data = lidar.getPointCloud()
        # Armazena as leituras na lista
        print(f"adicionei leituras:{len(lidar_readings)}")
        lidar_readings.append(lidar_data)
        key: int = kb.getKey()
        if key == ord('W'):
            cmd_vel(robot, keyboard_linear_vel, 0)
        elif key == ord('S'):
            cmd_vel(robot, -keyboard_linear_vel, 0)
        elif key == ord('A'):
            cmd_vel(robot, 0, keyboard_angular_vel)
        elif key == ord('D'):
            cmd_vel(robot, 0, -keyboard_angular_vel)
        else:  # Not a movement key
            cmd_vel(robot, 0, 0)
            if key == ord(' '):
                lidar_data_processed = []
                for data in lidar_readings:
                    for data_point in data:
                        x = data_point.x
                        y = data_point.y
                        z = data_point.z
                        # Verifica se as coordenadas são finitas antes de adicionar à matriz
                        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                            lidar_data_processed.append([x, y, z])

                lidar_data_processed = np.array(lidar_data_processed)

                find_possible_poses_from_csv(lidar_data_processed, 4)

                plt.show()

if __name__ == '__main__':
    main()
